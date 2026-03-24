# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""
Script to perform a straight line test with simplified physics.
"""

import argparse
from isaaclab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(description="Straight line test for F1Tenth.")
parser.add_argument("--num_envs", type=int, default=1, help="Number of environments to spawn.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest of the imports."""
import torch
import os
import sys
import numpy as np

# Add arcproLab to sys.path
sys.path.append(os.path.join(os.path.dirname(__file__), ".."))

from isaaclab.envs import ManagerBasedRLEnv
from arcpro_env_cfg import ARCProEnvCfg

def main():
    # setup configuration
    env_cfg = ARCProEnvCfg()
    env_cfg.scene.num_envs = args_cli.num_envs
    # Extreme stability
    env_cfg.decimation = 10
    env_cfg.__post_init__() 
    
    # setup environment
    env = ManagerBasedRLEnv(cfg=env_cfg)
    
    # reset environment
    obs, _ = env.reset()
    
    # simulation loop
    count = 0
    max_steps = 200
    
    print("\n--- Starting Straight Line Test ---")
    
    while simulation_app.is_running() and count < max_steps:
        with torch.no_grad():
            actions = torch.zeros((env.num_envs, 6), device=env.device)
            
            # Command 10.0 rad/s to all wheels
            target = 10.0
            actions[:, 2] = -target # FL
            actions[:, 3] = target # FR
            actions[:, 4] = -target # RL
            actions[:, 5] = target # RR
            
            # step environment
            obs, _, _, _, _ = env.step(actions)
            
            # Log telemetry for env 0
            if count % 10 == 0:
                 pos = env.scene["robot"].data.root_pos_w[0]
                 vel = env.scene["robot"].data.root_lin_vel_w[0]
                 jv = env.scene["robot"].data.joint_vel[0, [25, 24, 16, 13]]
                 print(f"Step {count:4d} | Pos: ({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}) | Vel: {torch.linalg.norm(vel):.3f}")
                 print(f"          | JVs: {jv.cpu().numpy()}")
        
        count += 1

    # close environment
    env.close()

if __name__ == "__main__":
    main()
