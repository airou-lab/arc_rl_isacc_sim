# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import argparse
from isaaclab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(description="Check initial lateral error.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import os
import sys
import torch
import numpy as np

# Add arcproLab to sys.path
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
ARCPRO_LAB_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, ".."))
if ARCPRO_LAB_DIR not in sys.path:
    sys.path.append(ARCPRO_LAB_DIR)

from isaaclab.envs import ManagerBasedRLEnv
from arcpro_env_cfg import ARCProEnvCfg
from mdp.track_manager import get_track_manager
import omni.usd
from pxr import Usd, UsdGeom, UsdShade, Gf

def main():
    # 1. Setup Environment
    env_cfg = ARCProEnvCfg()
    env_cfg.scene.num_envs = 1
    env_cfg.enable_cameras = False
    env_cfg.observations.visual = None
    env_cfg.__post_init__()

    env = ManagerBasedRLEnv(cfg=env_cfg)
    env.reset()
    
    # 2. Get Math Data
    asset = env.scene["robot"]
    pos = asset.data.root_pos_w - env.scene.env_origins
    q = asset.data.root_quat_w
    yaw = torch.atan2(2.0 * (q[:, 0] * q[:, 3] + q[:, 1] * q[:, 2]), 1.0 - 2.0 * (q[:, 2]**2 + q[:, 3]**2))
    
    tm = get_track_manager(device=env.device)
    lat_err, _ = tm.compute_errors(pos, yaw)
    closest_wp = tm.get_closest_waypoint_data(pos)
    
    print(f"\n--- Initial State Audit ---")
    print(f"Robot Local Pos: {pos[0, :2].cpu().numpy()}")
    print(f"Closest WP Pos: {closest_wp[0, :2].cpu().numpy()}")
    print(f"Initial LatErr: {lat_err[0].item():.4f}m")
    
    # 3. Find Physical Yellow Line in USD
    stage = omni.usd.get_context().get_stage()
    yellow_x = []
    for prim in stage.Traverse():
        path = str(prim.GetPath())
        if "/World/envs/env_0/Track" in path and "yellow" in path.lower() and prim.IsA(UsdGeom.Mesh):
            xform = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
            points = UsdGeom.Mesh(prim).GetPointsAttr().Get()
            for p in points:
                p_world = xform.Transform(p)
                # Only look at points near spawn Y (5.56)
                if abs(p_world[1] - 5.56) < 0.5:
                    yellow_x.append(p_world[0])
    
    if yellow_x:
        avg_yellow_x = np.mean(yellow_x)
        print(f"Physical Yellow Line X (near spawn): {avg_yellow_x:.4f}")
        print(f"Calculated Offset (Robot X - Yellow X): {pos[0, 0].item() - avg_yellow_x:.4f}m")
    else:
        print("Could not find Yellow Line mesh near spawn.")

    env.close()
    simulation_app.close()

if __name__ == "__main__":
    main()
