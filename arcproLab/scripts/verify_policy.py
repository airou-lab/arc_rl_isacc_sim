# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""
Script to verify the SB3 policy in the Isaac Lab environment.
"""

import argparse
from isaaclab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(description="Verify the SB3 policy in the Isaac Lab environment.")
parser.add_argument("--num_envs", type=int, default=1, help="Number of environments to spawn.")
parser.add_argument("--max_steps", type=int, default=10000, help="Maximum number of simulation steps.")
parser.add_argument("--checkpoint", type=str, required=True, help="Path to the SB3 checkpoint (.zip).")
# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
# parse the arguments
args_cli = parser.parse_args()
args_cli.enable_cameras = True # Hardcode for TiledCamera support

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest of the imports."""
import torch
import os
import sys
import numpy as np
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import VecNormalize

# Add both root and arcproLab to sys.path
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, "..", ".."))
ARCPRO_LAB_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, ".."))

if ROOT_DIR not in sys.path:
    sys.path.append(ROOT_DIR)
if ARCPRO_LAB_DIR not in sys.path:
    sys.path.append(ARCPRO_LAB_DIR)

from isaaclab.envs import ManagerBasedRLEnv
from arcpro_env_cfg import ARCProEnvCfg
from isaaclab_rl.sb3 import Sb3VecEnvWrapper

def main():
    # setup configuration
    env_cfg = ARCProEnvCfg()
    env_cfg.scene.num_envs = args_cli.num_envs
    env_cfg.enable_cameras = True 
    env_cfg.__post_init__() 
    
    # setup environment
    print("Initializing ManagerBasedRLEnv...")
    env = ManagerBasedRLEnv(cfg=env_cfg)
    
    # Wrap for SB3
    print("Wrapping environment for SB3...")
    env = Sb3VecEnvWrapper(env)
    
    # Add Normalization (Must match training settings)
    print("Adding observation normalization...")
    env = VecNormalize(env, norm_obs=True, norm_reward=True, clip_obs=10.)
    
    # Load model
    print(f"Loading SB3 policy from: {args_cli.checkpoint}")
    # Note: Map location to cuda if available
    model = PPO.load(args_cli.checkpoint, env=env, device="cuda" if torch.cuda.is_available() else "cpu")
    print("Policy loaded.")
    
    # reset environment
    print("Resetting environment...")
    obs = env.reset()
    print("Environment reset complete.")

    # simulation loop
    count = 0
    max_steps = args_cli.max_steps
    print(f"Starting simulation loop for {max_steps} steps...")
    
    try:
        while simulation_app.is_running() and count < max_steps:
            # Predict action
            actions, _ = model.predict(obs, deterministic=True)
            
            # Step environment
            obs, rewards, dones, infos = env.step(actions)
            
            count += 1
            if count % 100 == 0:
                print(f"Step {count}/{max_steps}")
    except KeyboardInterrupt:
        print("Simulation interrupted by user.")
    finally:
        print("Closing environment...")
        env.close()
        simulation_app.close()

if __name__ == "__main__":
    main()
