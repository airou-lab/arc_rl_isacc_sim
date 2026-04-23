# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Test intersection permeability.")
parser.add_argument("--test_mode", type=str, default="gate", choices=["gate", "wall"], help="Test crossing a gate or hitting a wall.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
args_cli.headless = False # Force GUI
args_cli.enable_cameras = True

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import torch
import os
import sys
import numpy as np

# Add arcproLab to sys.path
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
ARCPRO_LAB_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, ".."))
if ARCPRO_LAB_DIR not in sys.path:
    sys.path.insert(0, ARCPRO_LAB_DIR)

from arcpro_env_cfg import ARCProEnvCfg
from isaaclab.envs import ManagerBasedRLEnv
from mdp.track_manager import get_track_manager

def main():
    # 1. Setup environment
    env_cfg = ARCProEnvCfg()
    env_cfg.scene.num_envs = 1
    env_cfg.__post_init__()
    env = ManagerBasedRLEnv(cfg=env_cfg)

    # 2. Get TrackManager
    tm = get_track_manager(device=env.device)
    tm.debug = True # Force debug spheres for this test
    tm.ensure_synced()

    # 3. Find a test location
    # Nearest Gate to default spawn point
    default_spawn = np.array([-16.198, 5.45, 0.14])
    if tm.gate_tensor is not None and len(tm.gate_tensor) > 0:
        gate_pts = tm.gate_tensor.cpu().numpy()
        dists = np.linalg.norm(gate_pts[:, :2] - default_spawn[:2], axis=1)
        nearest_idx = np.argmin(dists)
        target_gate = gate_pts[nearest_idx]
        print(f"[TEST] Found target gate at: {target_gate} (Dist to spawn: {dists[nearest_idx]:.2f}m)")
    else:
        print("CRITICAL: No gates found to test!")
        return

    # 4. Simulation Loop
    count = 0
    test_started = False
    
    while simulation_app.is_running():
        # Step the environment
        # Action: [Steer, Throttle, Brake]
        # We'll just drive straight
        action = torch.tensor([[0.0, 0.8, 0.0]], device=env.device)
        obs, rewards, terminated, truncated, info = env.step(action)
        
        # Teleport once at the start to the gate
        if not test_started:
            print("[TEST] Teleporting robot to intersection...")
            # Position it 2.5m behind the gate, facing it (South: -1.57 rad)
            # Offset x by +0.3 to get away from yellow centerline
            spawn_pos = torch.tensor([[target_gate[0] + 0.3, target_gate[1] + 2.5, 0.15]], device=env.device)
            spawn_quat = torch.tensor([[0.707, 0.0, 0.0, -0.707]], device=env.device) # Facing South
            
            # If testing 'wall', offset it to the side to hit yellow line
            if args_cli.test_mode == "wall":
                 spawn_pos[0, 0] -= 0.3 # Move left toward yellow line
            
            env.scene["robot"].write_root_pose_to_sim(torch.cat([spawn_pos, spawn_quat], dim=-1))
            env.scene["robot"].write_root_velocity_to_sim(torch.zeros((1, 6), device=env.device))
            test_started = True

        if terminated[0]:
            print(f"!!! TERMINATED at step {count} !!!")
            if args_cli.test_mode == "wall":
                print("SUCCESS: Robot correctly reset when hitting the WALL.")
            else:
                print("FAILURE: Robot reset when it should have passed through the GATE.")
            break
            
        if count > 200:
            print(f"--- Completed {count} steps without reset ---")
            if args_cli.test_mode == "gate":
                print("SUCCESS: Robot successfully passed through the GATE!")
            else:
                print("FAILURE: Robot failed to reset on the WALL.")
            break

        count += 1
        simulation_app.update()

    env.close()
    simulation_app.close()

if __name__ == "__main__":
    main()
