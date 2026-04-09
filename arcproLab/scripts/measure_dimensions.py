# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Dimension measurement diagnostic.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../..")))

import torch
import omni.usd
from pxr import UsdGeom, Gf
from isaaclab.envs import ManagerBasedRLEnv
from arcproLab.arcpro_env_cfg import ARCProEnvCfg

def main():
    env_cfg = ARCProEnvCfg()
    env_cfg.scene.num_envs = 1
    env = ManagerBasedRLEnv(cfg=env_cfg)
    
    stage = omni.usd.get_context().get_stage()
    
    print("\n" + "="*60)
    print("PHYSICAL DIMENSION MEASUREMENT")
    print("="*60 + "\n")

    # 1. Measure Robot Width (Visual Mesh)
    robot_prim = stage.GetPrimAtPath("/World/envs/env_0/Robot/Chassis")
    if robot_prim.IsValid():
        bbox = UsdGeom.Imageable(robot_prim).ComputeWorldBound(0, "default")
        range = bbox.GetRange()
        min_pt, max_pt = range.GetMin(), range.GetMax()
        width = max_pt[1] - min_pt[1]
        length = max_pt[0] - min_pt[0]
        print(f"Robot (8x Scale):")
        print(f"  - Width:  {width:.3f} meters")
        print(f"  - Length: {length:.3f} meters")
    
    # 2. Measure Lane Width (Road Mesh near spawn)
    # We sample the pavement mesh near the spawn point
    road_prim = stage.GetPrimAtPath("/World/envs/env_0/Track/drivable_surfaces/tiles/pavement_1/ref_pavement_1/road_tile_1/opaque__asphalt__drivable_light_Road/piece_1")
    if road_prim.IsValid():
        bbox = UsdGeom.Imageable(road_prim).ComputeWorldBound(0, "default")
        range = bbox.GetRange()
        min_pt, max_pt = range.GetMin(), range.GetMax()
        # In this segment, the road is aligned along the X axis? No, let's check both.
        dx = max_pt[0] - min_pt[0]
        dy = max_pt[1] - min_pt[1]
        print(f"Road Tile Dimensions:")
        print(f"  - DX: {dx:.3f}m, DY: {dy:.3f}m")
        print(f"  - If Lane 1 + Lane 2 = {dy:.3f}m, then one Lane is {dy/2:.3f}m wide.")

    print("\n" + "="*60)
    env.close()
    simulation_app.close()

if __name__ == "__main__":
    main()
