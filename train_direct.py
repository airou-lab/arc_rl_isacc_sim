#!/usr/bin/env python3
"""
train_direct.py - High-Fidelity Metric Retraining for ARCPro
Uses IsaacDirectEnv (Fast API) + RecurrentPPO
"""

import os
import sys
import argparse
from datetime import datetime
from pathlib import Path
import numpy as np
import torch

# 1. Initialize SimulationApp FIRST
print("[Train] Initializing Isaac Sim...")
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

# 2. Imports after SimulationApp
from sb3_contrib import RecurrentPPO
from stable_baselines3.common.callbacks import CheckpointCallback, CallbackList, BaseCallback
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.vec_env import DummyVecEnv

# Path setup for imports
POLICY_DIR = "/home/arika/Documents/arcpro/arcpro_system/src/examples/ARCPro_RL/arc_rl_isacc_policy"
if POLICY_DIR not in sys.path:
    sys.path.append(POLICY_DIR)

from isaac_direct_env import IsaacDirectEnv, IsaacDirectConfig
from policies.hierarchical_policy import HierarchicalPathPlanningPolicy
from policies.fusion_policy import FusionFeaturesExtractor

class TrainingStatsCallback(BaseCallback):
    """Log distance and speed metrics to TensorBoard."""
    def __init__(self, verbose: int = 0):
        super().__init__(verbose)

    def _on_step(self) -> bool:
        infos = self.locals.get("infos", [])
        for info in infos:
            if "episode" in info:
                # Note: IsaacDirectEnv puts distance in telemetry vec[11]
                # but Monitor puts standard SB3 episode stats in info['episode']
                pass
        return True

def main():
    parser = argparse.ArgumentParser(description="ARCPro High-Fidelity Retraining")
    parser.add_argument("--timesteps", type=int, default=200_000)
    parser.add_argument("--name", type=str, default=None)
    parser.add_argument("--save-freq", type=int, default=25_000)
    parser.add_argument("--lr", type=float, default=3e-4)
    parser.add_argument("--device", type=str, default="auto")
    args = parser.parse_args()

    if args.name is None:
        args.name = datetime.now().strftime("metric_hppo_%Y%m%d_%H%M%S")

    # Dir setup
    run_dir = Path("models") / args.name
    run_dir.mkdir(parents=True, exist_ok=True)
    ckpt_dir = run_dir / "checkpoints"
    ckpt_dir.mkdir(exist_ok=True)
    tb_dir = run_dir / "tb"

    print(f"[Train] Experiment: {args.name}")
    print(f"[Train] Run directory: {run_dir}")

    # Environment
    config = IsaacDirectConfig(headless=True)
    
    def make_env():
        env = IsaacDirectEnv(config=config, simulation_app=simulation_app)
        env = Monitor(env)
        return env

    vec_env = DummyVecEnv([make_env])

    # Policy Configuration
    policy_kwargs = dict(
        features_extractor_class=FusionFeaturesExtractor,
        features_extractor_kwargs=dict(features_dim=268),
        lstm_hidden_size=256,
        n_lstm_layers=1,
        enable_critic_lstm=True,
        share_features_extractor=True,
        num_waypoints=5,
        waypoint_horizon=1.0, # 1.0m horizon for 40cm car
        use_kinematic_anchors=True,
    )

    model = RecurrentPPO(
        policy=HierarchicalPathPlanningPolicy,
        env=vec_env,
        learning_rate=args.lr,
        n_steps=128,
        batch_size=128,
        n_epochs=10,
        gamma=0.99,
        gae_lambda=0.95,
        clip_range=0.2,
        ent_coef=0.01,
        verbose=1,
        device=args.device,
        tensorboard_log=str(tb_dir),
        policy_kwargs=policy_kwargs,
    )

    # Callbacks
    callbacks = CallbackList([
        CheckpointCallback(
            save_freq=args.save_freq,
            save_path=str(ckpt_dir),
            name_prefix="hppo",
            verbose=1,
        ),
        TrainingStatsCallback()
    ])

    # Train
    try:
        print(f"[Train] Starting learning for {args.timesteps} steps...")
        model.learn(
            total_timesteps=args.timesteps,
            callback=callbacks,
            progress_bar=True,
        )
    except KeyboardInterrupt:
        print("\n[Train] Interrupted by user.")
    finally:
        # Save final model
        final_path = run_dir / "final_model"
        model.save(str(final_path))
        print(f"[Train] Final model saved to: {final_path}.zip")
        
        vec_env.close()
        simulation_app.close()

if __name__ == "__main__":
    main()
