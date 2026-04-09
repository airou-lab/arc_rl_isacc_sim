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
    env = VecNormalize(env, norm_obs=True, norm_reward=True, clip_obs=10.)

    # 5. Define Log Directory
    log_dir = os.path.join("logs", "ppo", datetime.now().strftime("%Y%m%d-%H%M%S"))
    os.makedirs(log_dir, exist_ok=True)

    # 6. Define Policy & Model
    # Note: Using MlpPolicy as Sb3VecEnvWrapper currently flattens observations.
    model = PPO(
        "MlpPolicy",
        env,
        verbose=1,
        learning_rate=3e-4,
        n_steps=2048,
        batch_size=64,
        n_epochs=10,
        gamma=0.99,
        gae_lambda=0.95,
        clip_range=0.2,
        ent_coef=0.0,
        tensorboard_log=log_dir,
        seed=args_cli.seed,
        device="cuda"
    )

    # 7. Callbacks
    checkpoint_callback = CheckpointCallback(save_freq=10000, save_path=log_dir, name_prefix="model")

    # 8. Train
    print(f"Starting training for {args_cli.total_timesteps} steps...")
    model.learn(
        total_timesteps=args_cli.total_timesteps,
        callback=checkpoint_callback,
        progress_bar=True
    )

    # 9. Save Final Model
    model.save(os.path.join(log_dir, "model_final"))
    env.save(os.path.join(log_dir, "vec_normalize.pkl"))
    
    # 10. Close
    env.close()

if __name__ == "__main__":
    main()
