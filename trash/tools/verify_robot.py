# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Script to verify the physical and kinematic properties of the ARCPro robot."""

import argparse
from isaaclab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(description="Verify the physical and kinematic properties of the ARCPro robot.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import torch
import numpy as np
from pxr import Usd, UsdGeom, Gf
import omni.usd

import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), ".."))

from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
from arcpro_env_cfg import ARCProSceneCfg

def main():
    # Setup scene
    scene_cfg = ARCProSceneCfg(num_envs=1, env_spacing=5.0)
    scene = InteractiveScene(scene_cfg)
    
    # Play simulation to initialize physics
    simulation_app.update()
    
    robot = scene["robot"]
    print("\n" + "="*50)
    print("      ARCPro Robot Physical Audit")
    print("="*50)
    
    # 1. Mass Audit
    # We set mass=4.092 in ARCPRO_ROBOT_CFG.spawn.mass_props
    # In Isaac Lab, this is applied to the root prim if not specified otherwise
    root_mass = robot.root_physx_view.get_masses()[0].item()
    print(f"[Audit] Total Root Mass: {root_mass:.4f} kg (Target: 4.092 kg)")
    
    # 2. Kinematic Audit (Wheelbase and Track Width)
    # Get joint positions to verify wheelbase
    # We'll use the relative positions of the wheels
    wheel_names = [
        "Wheel__Knuckle__Front_Left", 
        "Wheel__Knuckle__Front_Right", 
        "Wheel__Upright__Rear_Left", 
        "Wheel__Upright__Rear_Right"
    ]
    
    wheel_positions = {}
    stage = omni.usd.get_context().get_stage()
    
    for name in wheel_names:
        # Find prim in env_0
        prim_path = f"/World/envs/env_0/Robot/Rigid_Bodies/{name.split('__')[-1]}" # Simplified path logic
        # Actually let's search for it
        found_path = None
        for prim in Usd.PrimRange(stage.GetPrimAtPath("/World/envs/env_0/Robot")):
            if name in prim.GetPath().pathString:
                found_path = prim.GetPath().pathString
                break
        
        if found_path:
            xform = UsdGeom.Xformable(stage.GetPrimAtPath(found_path))
            world_transform = xform.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
            translation = world_transform.ExtractTranslation()
            wheel_positions[name] = np.array([translation[0], translation[1], translation[2]])

    if len(wheel_positions) == 4:
        fl = wheel_positions["Wheel__Knuckle__Front_Left"]
        fr = wheel_positions["Wheel__Knuckle__Front_Right"]
        rl = wheel_positions["Wheel__Upright__Rear_Left"]
        rr = wheel_positions["Wheel__Upright__Rear_Right"]
        
        # Wheelbase: Front axle center to Rear axle center
        front_axle = (fl + fr) / 2.0
        rear_axle = (rl + rr) / 2.0
        wheelbase = np.linalg.norm(front_axle[:2] - rear_axle[:2])
        
        # Track width: distance between wheels on same axle
        front_track = np.linalg.norm(fl[:2] - fr[:2])
        rear_track = np.linalg.norm(rl[:2] - rr[:2])
        
        print(f"[Audit] Wheelbase: {wheelbase*100:.2f} cm (Target: 25.0 cm)")
        print(f"[Audit] Track Width (Front): {front_track*100:.2f} cm (Target: 24.0 cm)")
        print(f"[Audit] Track Width (Rear): {rear_track*100:.2f} cm")
    else:
        print("[Audit] ERR: Could not locate all 4 wheels for kinematic audit.")

    # 3. Dimension Audit (Bounding Box)
    robot_prim = stage.GetPrimAtPath("/World/envs/env_0/Robot")
    bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    bbox = bbox_cache.ComputeWorldBound(robot_prim)
    range = bbox.GetRange()
    min_pt = range.GetMin()
    max_pt = range.GetMax()
    
    length = max_pt[0] - min_pt[0]
    width = max_pt[1] - min_pt[1]
    height = max_pt[2] - min_pt[2]
    
    print(f"[Audit] Bounding Box Length: {length*1000:.2f} mm (Target: 403 mm)")
    print(f"[Audit] Bounding Box Width: {width*1000:.2f} mm (Target: 287 mm)")
    print(f"[Audit] Bounding Box Height: {height*1000:.2f} mm (Target: 330 mm)")

    # 4. Sensor Audit
    camera_prim = stage.GetPrimAtPath("/World/envs/env_0/Robot/Rigid_Bodies/Chassis/Camera_RGB")
    if camera_prim:
        xform = UsdGeom.Xformable(camera_prim)
        world_transform = xform.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        cam_pos = world_transform.ExtractTranslation()
        print(f"[Audit] Camera Height (Ground): {cam_pos[2]*1000:.2f} mm (Target: 195 mm)")
    else:
        print("[Audit] ERR: Camera_RGB prim not found.")

    print("="*50 + "\n")
    
    # Close the simulator
    simulation_app.close()

if __name__ == "__main__":
    main()
