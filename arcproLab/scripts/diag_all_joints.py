# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""
Extreme diagnostic script: Actuate ALL joints.
"""

import argparse
from isaaclab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(description="Extreme diagnostic.")
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
    
    print(f"Action dim: {env.action_manager.total_action_dim}")
    
    # simulation loop
    count = 0
    max_steps = 200
    while simulation_app.is_running() and count < max_steps:
        with torch.no_grad():
            actions = torch.ones((env.num_envs, env.action_manager.total_action_dim), device=env.device) * 5.0
            
            # step environment
            obs, _, _, _, _ = env.step(actions)
            
            # Log telemetry for env 0
            if count % 20 == 0:
                 jv = env.scene["robot"].data.joint_vel[0]
                 print(f"Step {count:4d} | Non-zero velocities:")
                 for i, val in enumerate(jv):
                     if abs(val) > 0.1:
                         print(f"  {i} ({env.scene['robot'].data.joint_names[i]}): {val:.3f}")
        
        count += 1

    # close environment
    env.close()

if __name__ == "__main__":
    main()
