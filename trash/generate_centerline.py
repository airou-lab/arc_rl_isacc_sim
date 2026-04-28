# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""
World-Vector Path Generator.
"""

import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--num_envs", type=int, default=1)
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import os
import sys
sys.path.append(os.path.join(os.path.dirname(__file__), ".."))

import numpy as np
from isaaclab.envs import ManagerBasedRLEnv
from arcpro_env_cfg import ARCProEnvCfg
from mdp.track_manager import get_track_manager

def main():
    env_cfg = ARCProEnvCfg()
    env_cfg.scene.num_envs = 1
    env_cfg.enable_cameras = False 
    env = ManagerBasedRLEnv(cfg=env_cfg)

    tm = get_track_manager(device=env.device)
    tm.ensure_synced()
    
    y_pts = tm.raw_yellow_pts
    w_pts = tm.raw_white_pts
    
    if y_pts is None or w_pts is None:
        print("[ERROR] Boundaries empty.")
        return

    # 1. Generate Mids (2D)
    raw_mids = []
    for p_y in y_pts:
        dists = np.linalg.norm(w_pts[:, :2] - p_y[:2], axis=1)
        idx_w = np.argmin(dists)
        if 0.1 < dists[idx_w] < 1.0:
            raw_mids.append((p_y[:2] + w_pts[idx_w][:2]) / 2.0)
    
    raw_mids = np.unique(np.round(np.array(raw_mids), 3), axis=0)

    # 2. Sequential Sorting (Greedy)
    # Start at the point closest to spawn X=-16.2, Y=5.4
    start_p = np.array([-16.2, 5.4])
    curr_idx = np.argmin(np.linalg.norm(raw_mids - start_p, axis=1))
    
    chain = [raw_mids[curr_idx]]
    visited = {curr_idx}
    
    for _ in range(len(raw_mids)):
        curr_p = chain[-1]
        dists = np.linalg.norm(raw_mids - curr_p, axis=1)
        dists[list(visited)] = 1e6
        next_idx = np.argmin(dists)
        if dists[next_idx] < 0.5:
            chain.append(raw_mids[next_idx])
            visited.add(next_idx)
        else: break

    chain = np.array(chain)
    
    # 3. Add Yaw and Save
    final_data = []
    for i in range(len(chain) - 1):
        p1, p2 = chain[i], chain[i+1]
        yaw = np.arctan2(p2[1] - p1[1], p2[0] - p1[0])
        final_data.append([p1[0], p1[1], yaw])
            
    final_data = np.array(final_data)
    np.save(os.path.join(os.path.dirname(__file__), "../mdp/track_centerline_1x.npy"), final_data)
    
    print(f"\n[SUCCESS] Generated {len(final_data)} waypoints.")
    env.close()

if __name__ == "__main__":
    main()
