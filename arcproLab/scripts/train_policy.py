# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import argparse
from isaaclab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(description="Train an SB3 policy for ARCPro Lane Following.")
parser.add_argument("--num_envs", type=int, default=16, help="Number of parallel simulation environments.")
parser.add_argument("--seed", type=int, default=42, help="Seed for the environment.")
parser.add_argument("--total_timesteps", type=int, default=1000000, help="Total timesteps to train.")
parser.add_argument("--checkpoint", type=str, default=None, help="Path to a checkpoint to resume from.")
# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
# parse the arguments
args_cli = parser.parse_args()

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import os
import sys
import torch
import warnings
from datetime import datetime

# Silence gym warnings
warnings.filterwarnings("ignore", category=DeprecationWarning)

# Add both root and arcproLab to sys.path
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, "..", ".."))
ARCPRO_LAB_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, ".."))

if ROOT_DIR not in sys.path:
    sys.path.append(ROOT_DIR)
if ARCPRO_LAB_DIR not in sys.path:
    sys.path.append(ARCPRO_LAB_DIR)

from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import CheckpointCallback, EvalCallback
from stable_baselines3.common.vec_env import VecNormalize

from isaaclab.envs import ManagerBasedRLEnv
from arcpro_env_cfg import ARCProEnvCfg
from isaaclab_rl.sb3 import Sb3VecEnvWrapper
from policy_stack.policies.fusion_policy import FusionFeaturesExtractor

def main():
    # 1. Setup Environment Configuration
    env_cfg = ARCProEnvCfg()
    env_cfg.scene.num_envs = args_cli.num_envs
    # Enable cameras for vision-based training
    env_cfg.enable_cameras = True 
    env_cfg.__post_init__() 

    # 2. Create Environment
    env = ManagerBasedRLEnv(cfg=env_cfg)
    
    # 3. Wrap for SB3
    env = Sb3VecEnvWrapper(env)
    
    # 4. Add Normalization
    #
    # `norm_obs_keys=["telemetry"]` is critical: by default VecNormalize on a
    # Dict obs normalizes EVERY key including `tiled_camera`, which would
    # convert the uint8 image to running-mean/std float32 and break the
    # pretrained ResNet's input pipeline (it expects uint8 -> SB3 /255 ->
    # ImageNet mean/std). Telemetry still gets normalized, which is what we
    # want for the 12-D heterogeneous-scale vector.
    env = VecNormalize(
        env,
        norm_obs=True,
        norm_obs_keys=["telemetry"],
        norm_reward=True,
        clip_obs=10.0,
    )

    # 5. Define Log Directory
    log_dir = os.path.join("logs", "ppo", datetime.now().strftime("%Y%m%d-%H%M%S"))
    os.makedirs(log_dir, exist_ok=True)

    # 6. Define Policy & Model
    if args_cli.checkpoint:
        print(f"Resuming from checkpoint: {args_cli.checkpoint}")
        model = PPO.load(
            args_cli.checkpoint,
            env,
            verbose=1,
            tensorboard_log=None,
            seed=args_cli.seed,
            device="cuda"
        )
    else:
        # MultiInputPolicy handles the Dict observation (telemetry + tiled_camera)
        # natively. We override SB3's default CombinedExtractor with
        # FusionFeaturesExtractor: ImageNet-pretrained ResNet-18 on the image
        # (224x224x3 uint8 -> 512-d feats -> Linear(512,256)+ReLU), identity
        # passthrough on the 12-D telemetry, concatenate + LayerNorm to 268-d.
        # Requires PolicyCfg.concatenate_terms=False in arcpro_env_cfg.py.
        policy_kwargs = dict(
            features_extractor_class=FusionFeaturesExtractor,
            features_extractor_kwargs=dict(
                pretrained=True,
                cifar_stem=False,
                freeze_backbone=False,
                image_key="tiled_camera",
                vec_key="telemetry",
            ),
            # FusionFeaturesExtractor emits 268-d already; let the SB3 actor
            # / critic heads run their default MLPs on top.
        )
        model = PPO(
            "MultiInputPolicy",
            env,
            policy_kwargs=policy_kwargs,
            verbose=1,
            learning_rate=3e-4,
            n_steps=2048,
            batch_size=64,
            n_epochs=10,
            gamma=0.99,
            gae_lambda=0.95,
            clip_range=0.2,
            ent_coef=0.0,
            tensorboard_log=None,
            seed=args_cli.seed,
            device="cuda"
        )

    # 7. Callbacks
    from stable_baselines3.common.callbacks import BaseCallback
    
    class RewardLoggerCallback(BaseCallback):
        def __init__(self, verbose: int = 0):
            super().__init__(verbose)
        
        def _on_step(self) -> bool:
            if self.n_calls % 1000 == 0:
                # Get rewards from info
                info = self.locals.get("infos", [{}])[0]
                ep_rew = info.get("episode", {}).get("r", 0.0)
                ep_len = info.get("episode", {}).get("l", 0)
                print(f"[PROGRESS] Step {self.n_calls} | EpRew: {ep_rew:.2f} | EpLen: {ep_len}")
                sys.stdout.flush()
            return True

    class SaveVecNormalizeCallback(BaseCallback):
        def __init__(self, save_path: str, save_freq: int, verbose: int = 0):
            super().__init__(verbose)
            self.save_path = save_path
            self.save_freq = save_freq

        def _on_step(self) -> bool:
            if self.n_calls % self.save_freq == 0:
                self.training_env.save(os.path.join(self.save_path, "vec_normalize.pkl"))
            return True

    checkpoint_callback = CheckpointCallback(save_freq=5000, save_path=log_dir, name_prefix="model")
    vec_norm_callback = SaveVecNormalizeCallback(save_path=log_dir, save_freq=5000)
    reward_logger_callback = RewardLoggerCallback()

    # 8. Train
    print(f"Starting training for {args_cli.total_timesteps} steps...")
    model.learn(
        total_timesteps=args_cli.total_timesteps,
        callback=[checkpoint_callback, vec_norm_callback, reward_logger_callback],
        progress_bar=True
    )

    # 9. Save Final Model
    model.save(os.path.join(log_dir, "model_final"))
    env.save(os.path.join(log_dir, "vec_normalize.pkl"))
    
    # 10. Close
    env.close()

if __name__ == "__main__":
    main()
