# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Measure native USD sizes.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

from pxr import Usd, UsdGeom, Gf
import os

def get_native_size(usd_path, prim_path=None):
    stage = Usd.Stage.Open(usd_path)
    if not stage:
        return None
    
    # If no prim path, use pseudo-root children
    if prim_path:
        root_prim = stage.GetPrimAtPath(prim_path)
    else:
        root_prim = stage.GetPseudoRoot()
        
    bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    bbox = bbox_cache.ComputeWorldBound(root_prim)
    size = bbox.GetRange().GetMax() - bbox.GetRange().GetMin()
    return size

def main():
    robot_usd = "f1tenth_trainer/assets/F1Tenth_Modern.usd"
    track_usd = "openStreetUSD/arcpro_RL_open_street_sim.usd"
    
    # Measure robot (root is Full_Car)
    robot_size = get_native_size(robot_usd)
    print(f"\n[Native] Robot ({robot_usd}) Size: {robot_size[0]:.2f} x {robot_size[1]:.2f} x {robot_size[2]:.2f} units")
    
    # Measure road (find a representative tile)
    track_stage = Usd.Stage.Open(track_usd)
    tile_path = "/World/drivable_surfaces/tiles/pavement_1/ref_pavement_1/road_tile_1/opaque__asphalt__drivable_light_Road/piece_2"
    tile_prim = track_stage.GetPrimAtPath(tile_path)
    if tile_prim:
        bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
        bbox = bbox_cache.ComputeWorldBound(tile_prim)
        size = bbox.GetRange().GetMax() - bbox.GetRange().GetMin()
        print(f"[Native] Track Tile Size: {size[0]:.2f} x {size[1]:.2f} m")
    else:
        print("[Native] ERR: Could not find tile prim in Track USD.")

    simulation_app.close()

if __name__ == "__main__":
    main()
