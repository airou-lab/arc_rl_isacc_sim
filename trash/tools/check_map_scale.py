# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Script to check the scale of the map (road width) in relation to the robot."""

import argparse
from isaaclab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(description="Check the scale of the map in relation to the robot.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import torch
import numpy as np
import omni.usd
from pxr import Usd, UsdGeom, Gf

import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), ".."))

from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
from isaaclab.sim import SimulationContext, SimulationCfg
# Add project root and arcproLab to sys.path
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, "..", ".."))
ARCPRO_LAB_DIR = os.path.join(PROJECT_ROOT, "arcproLab")

if PROJECT_ROOT not in sys.path:
    sys.path.append(PROJECT_ROOT)
if ARCPRO_LAB_DIR not in sys.path:
    sys.path.append(ARCPRO_LAB_DIR)

from arcpro_env_cfg import ARCProSceneCfg

def main():
    # Setup simulation context
    sim_cfg = SimulationCfg(dt=1.0/60.0, device="cuda:0")
    sim = SimulationContext(sim_cfg)
    
    # Setup scene
    scene_cfg = ARCProSceneCfg(num_envs=1, env_spacing=5.0)
    # Disable camera to avoid rendering overhead
    scene_cfg.tiled_camera = None
    
    scene = InteractiveScene(scene_cfg)
    
    print("\n" + "="*50)
    print("      Map vs Robot Scale Audit")
    print("="*50)
    
    stage = omni.usd.get_context().get_stage()
    
    # 1. Robot Dimensions
    robot_prim = stage.GetPrimAtPath("/World/envs/env_0/Robot")
    bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    bbox_robot = bbox_cache.ComputeWorldBound(robot_prim)
    range_robot = bbox_robot.GetRange()
    robot_size = range_robot.GetMax() - range_robot.GetMin()
    robot_width = robot_size[1]
    robot_length = robot_size[0]
    
    print(f"[Robot] Length: {robot_length:.3f} m")
    print(f"[Robot] Width: {robot_width:.3f} m")
    
    # 2. Map Analysis (Road Width)
    print("[Map] Analyzing road meshes...")
    road_widths = []
    
    for prim in Usd.PrimRange(stage.GetPrimAtPath("/World/envs/env_0/Track")):
        if prim.IsA(UsdGeom.Mesh):
            prim_path = str(prim.GetPath())
            # Focus on 'road_tile' or 'pavement' which usually represents the main surface
            if "road_tile" in prim_path.lower() or "pavement" in prim_path.lower():
                bbox_road = bbox_cache.ComputeWorldBound(prim)
                r = bbox_road.GetRange()
                size = r.GetMax() - r.GetMin()
                # A road tile is typically a large square or rectangle. 
                # Its "width" is the cross-section. 
                # Let's just report the dimensions.
                print(f" - Found road segment: {prim_path}, Dims: {size[0]:.2f} x {size[1]:.2f} m")
                # Estimate width as the smaller dimension if it's a long segment, or larger if it's a cross section
                # For this specific map, tiles seem to be ~50-60m long and ~8-14m wide.
                w = min(size[0], size[1])
                if w > 1.0:
                    road_widths.append(w)

    if road_widths:
        avg_road_width = sum(road_widths) / len(road_widths)
        print(f"[Map] Typical Road/Pavement Width: {avg_road_width:.2f} m")
        
        # Relation
        ratio = avg_road_width / robot_width
        print(f"\n[Relation] Road is approx {ratio:.2f}x wider than the robot.")
        
        if ratio < 1.5:
            print("[Relation] WARNING: Road is very narrow for this robot!")
        elif ratio > 10.0:
            print("[Relation] WARNING: Robot is very small compared to the road!")
        else:
            print("[Relation] SUCCESS: Robot scale seems appropriate for the road width.")
    else:
        print("[Map] ERR: Could not find any road meshes to measure width.")

    print("="*50 + "\n")
    
    # Close the simulator
    simulation_app.close()

if __name__ == "__main__":
    main()
