# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Coordinate mismatch diagnostic.")
parser.add_argument("--num_envs", type=int, default=1, help="Number of environments.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

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
    env_cfg = ARCProEnvCfg()
    env_cfg.scene.num_envs = args_cli.num_envs
    env = ManagerBasedRLEnv(cfg=env_cfg)
    tm = get_track_manager(device=env.device)
    
    print("\n" + "="*60)
    print("COORDINATE MISMATCH CHECK")
    print("="*60 + "\n")

    env.reset()
    for count in range(10):
        obs, _, _, _, _ = env.step(torch.zeros(env.action_manager.action.shape, device=env.device))
        
        # 1. Get Robot World Position
        robot_pos = env.scene["robot"].data.root_pos_w[0]
        
        # 2. Get Closest Waypoint World Position
        wp_data = tm.get_closest_waypoint_data(env.scene["robot"].data.root_pos_w)
        wp_pos = wp_data[0, :2]
        
        # 3. Calculate Direct Distance
        dist = torch.norm(robot_pos[:2] - wp_pos).item()
        
        print(f"Step {count:2d}")
        print(f"  Robot World Pos:   X={robot_pos[0]:.3f}, Y={robot_pos[1]:.3f}")
        print(f"  Closest Waypoint:  X={wp_pos[0]:.3f}, Y={wp_pos[1]:.3f}")
        print(f"  Visual Mismatch:   {dist:.3f} meters")
        print("-" * 30)

    env.close()
    simulation_app.close()

if __name__ == "__main__":
    main()
