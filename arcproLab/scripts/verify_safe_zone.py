# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Verify the 1.8m-2.7m Safe Zone visually.")
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
    env_cfg.scene.num_envs = 1
    # Disable actual terminations for this test so we can watch the drift
    env_cfg.terminations.roadmark_contact = None
    env_cfg.terminations.height = None
    
    env = ManagerBasedRLEnv(cfg=env_cfg)
    tm = get_track_manager(device=env.device)
    
    print("\n" + "="*60)
    print("SAFE ZONE VISUAL VERIFICATION")
    print("Corridor: [1.8m (Yellow) <--> 2.7m (White)]")
    print("="*60 + "\n")

    # Spawn at the calculated Lane Center (X = -129.08)
    # Using the waypoint center as base
    env.reset()
    
    count = 0
    while simulation_app.is_running():
        # Slow forward drive
        actions = torch.zeros(env.action_manager.action.shape, device=env.device)
        actions[:, 2:6] = -20.0 # Slow constant speed
        
        obs, _, _, _, _ = env.step(actions)
        
        # Calculate Errors
        robot_pos = env.scene["robot"].data.root_pos_w
        q = env.scene["robot"].data.root_quat_w
        yaw = torch.atan2(2.0 * (q[:, 0] * q[:, 3] + q[:, 1] * q[:, 2]), 1.0 - 2.0 * (q[:, 2]**2 + q[:, 3]**2))
        
        lat_err, _ = tm.compute_errors(robot_pos, yaw)
        val = lat_err[0].item()
        
        status = "SAFE"
        if val < 1.8:
            status = "!!! TOUCHING YELLOW LINE !!!"
        elif val > 2.7:
            status = "!!! TOUCHING WHITE LINE !!!"
            
        if count % 10 == 0:
            print(f"Step {count:4d} | LatErr: {val:.3f}m | Status: {status}")
            
        count += 1
        if count > 1000: break

    env.close()
    simulation_app.close()

if __name__ == "__main__":
    main()
