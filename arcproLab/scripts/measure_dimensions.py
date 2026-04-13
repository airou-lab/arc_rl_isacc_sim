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
    env_cfg.enable_cameras = False # Disable cameras for headless measurement
    env_cfg.scene.tiled_camera = None # Force remove from scene
    env_cfg.observations.visual = None # Disable visual observations
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
        # World Bound
        bbox_world = UsdGeom.Imageable(road_prim).ComputeWorldBound(0, "default")
        range_world = bbox_world.GetRange()
        print(f"Road Tile World Range: {range_world.GetMin()} to {range_world.GetMax()}")
        
        # Local Bound
        bbox_local = UsdGeom.Imageable(road_prim).ComputeLocalBound(0, "default")
        range_local = bbox_local.GetRange()
        print(f"Road Tile Local Range: {range_local.GetMin()} to {range_local.GetMax()}")
        
        dx = range_world.GetMax()[0] - range_world.GetMin()[0]
        dy = range_world.GetMax()[1] - range_world.GetMin()[1]
        print(f"Road Tile Dimensions:")
        print(f"  - DX: {dx:.3f}m, DY: {dy:.3f}m")
        print(f"  - If Lane 1 + Lane 2 = {dy:.3f}m, then one Lane is {dy/2:.3f}m wide.")
        
        # Accumulated World Transform
        world_transform = UsdGeom.Xformable(road_prim).ComputeLocalToWorldTransform(0)
        print(f"Road Tile World Transform Matrix:\n{world_transform}")
        
        # Extract scale from matrix
        scale = [
            torch.linalg.norm(torch.tensor(world_transform[0][:3])),
            torch.linalg.norm(torch.tensor(world_transform[1][:3])),
            torch.linalg.norm(torch.tensor(world_transform[2][:3]))
        ]
        print(f"Extracted World Scale: {scale}")

    print("\n" + "="*60)
    env.close()
    simulation_app.close()

if __name__ == "__main__":
    main()
