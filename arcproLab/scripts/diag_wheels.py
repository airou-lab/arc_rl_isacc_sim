# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""
Script to perform wheel-by-wheel diagnostic for robot joint mapping.
"""

import argparse
from isaaclab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(description="Wheel diagnostic for F1Tenth.")
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
    env_cfg.__post_init__() 
    
    # setup environment
    env = ManagerBasedRLEnv(cfg=env_cfg)
    
    # reset environment
    obs, _ = env.reset()
    
    # simulation loop
    count = 0
    # Phase durations (in steps)
    phase_len = 100
    
    print("\n--- Starting Wheel Diagnostic ---")
    print("Joint names in order: ", env.scene["robot"].data.joint_names)
    
    while simulation_app.is_running() and count < phase_len * 4:
        with torch.no_grad():
            actions = torch.zeros((env.num_envs, 6), device=env.device)
            
            # Diagnostic phase logic
            phase = count // phase_len
            wheel_idx = phase + 2 # Indices 2, 3, 4, 5 are drive wheels
            actions[:, wheel_idx] = 1.0 # Command 1.0 (rad/s since scale=1.0)
            
            # step environment
            obs, _, _, _, _ = env.step(actions)
            
            # Log telemetry for env 0
            if count % 20 == 0:
                 vel = env.scene["robot"].data.root_lin_vel_w[0]
                 jv = env.scene["robot"].data.joint_vel[0]
                 print(f"Step {count:4d} | Phase {phase} (Commanding Index {wheel_idx})")
                 print(f"          | Root Vel: {vel.cpu().numpy()}")
                 print(f"          | Drive JVs: {jv[[25, 24, 16, 13]].cpu().numpy()}")
        
        count += 1

    # close environment
    env.close()

if __name__ == "__main__":
    main()
