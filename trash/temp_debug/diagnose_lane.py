# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""
Diagnostic script to map the 8x robot footprint against the lane boundaries.
"""

import argparse
from isaaclab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(description="Diagnostic script for lane boundaries.")
parser.add_argument("--num_envs", type=int, default=1, help="Number of environments to spawn.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../..")))

import torch
from isaaclab.envs import ManagerBasedRLEnv
from arcproLab.arcpro_env_cfg import ARCProEnvCfg
from mdp.track_manager import get_track_manager

def main():
    # 1. Setup Environment
    env_cfg = ARCProEnvCfg()
    env_cfg.scene.num_envs = args_cli.num_envs
    env = ManagerBasedRLEnv(cfg=env_cfg)
    
    # 2. Setup Track Manager
    tm = get_track_manager(device=env.device)
    
    print("\n" + "="*60)
    print("LANE BOUNDARY DIAGNOSTIC (8x SCALE)")
    print("Goal: Map 4 corners of the robot footprint to Lateral Error.")
    print("="*60 + "\n")

    # Robot footprint offsets (8x scale)
    # Approx 4.0m length, 2.4m width
    offsets = torch.tensor([
        [2.0,  1.2, 0.0],  # Front Left
        [2.0, -1.2, 0.0],  # Front Right
        [-2.0,  1.2, 0.0], # Rear Left
        [-2.0, -1.2, 0.0]  # Rear Right
    ], device=env.device)

    count = 0
    env.reset()
    
    while simulation_app.is_running() and count < 100:
        # Step with zero actions
        actions = torch.zeros(env.action_manager.action.shape, device=env.device)
        obs, rewards, terminations, truncations, extras = env.step(actions)
        
        # Get robot state
        robot = env.scene["robot"]
        pos_w = robot.data.root_pos_w # (num_envs, 3)
        quat_w = robot.data.root_quat_w # (num_envs, 4)
        
        # Compute Yaw
        q = quat_w
        yaw = torch.atan2(2.0 * (q[:, 0] * q[:, 3] + q[:, 1] * q[:, 2]), 1.0 - 2.0 * (q[:, 2]**2 + q[:, 3]**2))
        
        # Calculate errors for center
        lat_err_center, _ = tm.compute_errors(pos_w, yaw)
        
        # Calculate errors for 4 corners
        # Rotate offsets into world frame manually
        cos_y = torch.cos(yaw)
        sin_y = torch.sin(yaw)
        
        corner_results = []
        for i, name in enumerate(["FL", "FR", "RL", "RR"]):
            off = offsets[i]
            # Rotate 2D offset
            rx = off[0] * cos_y - off[1] * sin_y
            ry = off[0] * sin_y + off[1] * cos_y
            
            corner_pos = pos_w.clone()
            corner_pos[:, 0] += rx
            corner_pos[:, 1] += ry
            
            lat_err_corner, _ = tm.compute_errors(corner_pos, yaw)
            corner_results.append(f"{name}: {lat_err_corner[0].item():.3f}m")

        print(f"Step {count:3d} | Center LatErr: {lat_err_center[0].item():.3f}m")
        print(f"         | Corners -> {' | '.join(corner_results)}")
        
        if count % 10 == 0:
            print("-" * 40)
            
        count += 1

    env.close()
    simulation_app.close()

if __name__ == "__main__":
    main()
