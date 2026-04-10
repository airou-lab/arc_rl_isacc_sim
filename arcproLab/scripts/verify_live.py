# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import argparse
from isaaclab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(description="Verify the SB3 policy in the Isaac Lab environment.")
parser.add_argument("--num_envs", type=int, default=1, help="Number of environments to spawn.")
parser.add_argument("--checkpoint", type=str, required=True, help="Path to the model checkpoint.")
# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
# parse the arguments
args_cli = parser.parse_args()
args_cli.enable_cameras = True 

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import torch
import os
import sys
import numpy as np
from stable_baselines3 import PPO

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
    # 1. Setup configuration
    env_cfg = ARCProEnvCfg()
    env_cfg.scene.num_envs = args_cli.num_envs
    env_cfg.enable_cameras = True 
    env_cfg.__post_init__() 
    
    # 2. Create environment
    env = ManagerBasedRLEnv(cfg=env_cfg)
    # Wrap for SB3
    env = Sb3VecEnvWrapper(env)
    
    # 3. Load model
    print(f"Loading checkpoint: {args_cli.checkpoint}")
    # Note: Loading without VecNormalize might result in slightly different behavior
    # but at 20k-30k steps, the effect is usually minimal for "seeing" progress.
    model = PPO.load(args_cli.checkpoint, env=env)
    
    # 4. Reset environment
    obs = env.reset()
    
    # 5. Simulation loop
    print("Starting visual verification...")
    while simulation_app.is_running():
        with torch.no_grad():
            # Get action from policy
            actions, _ = model.predict(obs, deterministic=True)
            # Step environment
            obs, rewards, dones, info = env.step(actions)
            
        # Update simulation app
        simulation_app.update()

    # 6. Close environment
    env.close()
    simulation_app.close()

if __name__ == "__main__":
    main()
