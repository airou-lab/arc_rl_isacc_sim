# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""
Straight-line test script to verify termination boundaries.
Drives the robot forward at a constant speed with zero steering.
"""

import argparse
from isaaclab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(description="Straight-line test for ARCPro RL.")
parser.add_argument("--num_envs", type=int, default=1, help="Number of environments to spawn.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import os
import sys
sys.path.append(os.path.join(os.path.dirname(__file__), ".."))

import torch
from isaaclab.envs import ManagerBasedRLEnv
from arcpro_env_cfg import ARCProEnvCfg

def main():
    # 1. Create Environment
    env_cfg = ARCProEnvCfg()
    env_cfg.scene.num_envs = args_cli.num_envs
    env_cfg.enable_cameras = False # Not needed for straight line test
    
    # Initialize the environment
    env = ManagerBasedRLEnv(cfg=env_cfg)

    # 2. Run Test Loop
    print("\n[TEST] Starting Straight-Line Drive...")
    print("[TEST] Goal: Verify exactly where the robot resets when hitting boundaries.")
    
    count = 0
    while simulation_app.is_running():
        # Constant Action: [Steering=0.0, Throttle=0.5, Brake=0.0]
        actions = torch.zeros(env.num_envs, 3, device=env.device)
        actions[:, 1] = 0.5 # Constant 50% throttle
        
        # Step environment
        obs, rewards, terminations, truncations, extras = env.step(actions)
        
        if terminations.any():
            print(f" - Reset triggered at step {count}")
            
        count += 1

    # close the simulator
    env.close()

if __name__ == "__main__":
    main()
