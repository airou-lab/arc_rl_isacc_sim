# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""
Final diagnostic: Find the joints that are actually moving.
"""

import argparse
from isaaclab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(description="Find moving joints.")
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
    env_cfg.__post_init__() 
    
    # setup environment
    env = ManagerBasedRLEnv(cfg=env_cfg)
    
    # reset environment
    obs, _ = env.reset()
    
    # simulation loop
    count = 0
    max_steps = 100
    while simulation_app.is_running() and count < max_steps:
        with torch.no_grad():
            actions = torch.zeros((env.num_envs, 6), device=env.device)
            # Command 20.0 to all potential wheels
            actions[:, 2:6] = 20.0 
            
            # step environment
            obs, _, _, _, _ = env.step(actions)
            
            # Log telemetry
            if count == 50:
                 jv = env.scene["robot"].data.joint_vel[0]
                 names = env.scene["robot"].data.joint_names
                 print("\n--- Joint Velocities at Step 50 ---")
                 for i, name in enumerate(names):
                     print(f"  {i:2d} | {name:40s} | Vel: {jv[i]:.3f}")
        
        count += 1

    # close environment
    env.close()

if __name__ == "__main__":
    main()
