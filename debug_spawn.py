
import argparse
import torch
import os
import sys
import numpy as np

from isaaclab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(description="Debug spawn distances.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

from isaaclab.envs import ManagerBasedRLEnv
from arcproLab.arcpro_env_cfg import ARCProEnvCfg
from arcproLab.mdp.track_manager import get_track_manager

def main():
    env_cfg = ARCProEnvCfg()
    env_cfg.scene.num_envs = 1
    env_cfg.__post_init__()

    env = ManagerBasedRLEnv(cfg=env_cfg)
    
    # Get TrackManager and check distances
    tm = get_track_manager(device=env.device)
    asset = env.scene["robot"]
    
    # Force a reset to ensure spawn
    env.reset()
    
    # Get robot pos
    pos = asset.data.root_pos_w - env.scene.env_origins
    print(f"\n[DEBUG] Robot Spawn Pos: {pos[0]}")
    
    # Calculate distances
    dist_y, dist_w, dist_g = tm.compute_marker_distances(pos)
    print(f"[DEBUG] Distance to Yellow Line: {dist_y[0]:.4f}")
    print(f"[DEBUG] Distance to White Line:  {dist_w[0]:.4f}")
    print(f"[DEBUG] Distance to Gate:        {dist_g[0]:.4f}")
    
    # Check if this triggers termination
    term_w = dist_w < 0.30
    term_y = dist_y < 0.30
    print(f"[DEBUG] Triggers White Line Termination:  {term_w[0]}")
    print(f"[DEBUG] Triggers Yellow Line Termination: {term_y[0]}")

    if term_w[0] or term_y[0]:
        print("\n[CONCLUSION] The robot is spawning INSIDE the termination zone.")
    else:
        print("\n[CONCLUSION] The spawn point is SAFE. The issue is likely in the first step's action.")

    env.close()

if __name__ == "__main__":
    main()
