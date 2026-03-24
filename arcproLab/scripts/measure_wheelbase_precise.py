# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Measure native wheelbase from joints.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

from pxr import Usd, UsdGeom, Gf
import numpy as np

def main():
    usd_path = "f1tenth_trainer/assets/F1Tenth_Modern.usd"
    stage = Usd.Stage.Open(usd_path)
    
    joint_paths = [
        "/Full_Car/Joints/Wheel__Knuckle__Front_Left",
        "/Full_Car/Joints/Wheel__Knuckle__Front_Right",
        "/Full_Car/Joints/Wheel__Upright__Rear_Left",
        "/Full_Car/Joints/Wheel__Upright__Rear_Right"
    ]
    
    positions = {}
    for path in joint_paths:
        prim = stage.GetPrimAtPath(path)
        if prim:
            xform = UsdGeom.Xformable(prim)
            local_to_world = xform.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
            translation = local_to_world.ExtractTranslation()
            positions[path] = np.array(translation)
            print(f"[Native] Joint {path}: {positions[path]}")
            
    if len(positions) == 4:
        fl = positions["/Full_Car/Joints/Wheel__Knuckle__Front_Left"]
        fr = positions["/Full_Car/Joints/Wheel__Knuckle__Front_Right"]
        rl = positions["/Full_Car/Joints/Wheel__Upright__Rear_Left"]
        rr = positions["/Full_Car/Joints/Wheel__Upright__Rear_Right"]
        
        front_axle_center = (fl + fr) / 2.0
        rear_axle_center = (rl + rr) / 2.0
        
        wheelbase = np.linalg.norm(front_axle_center - rear_axle_center)
        print(f"\n[Native] Native Wheelbase: {wheelbase:.4f} units")
        
        # Calculate Native Track Width
        track_width = np.linalg.norm(fl - fr)
        print(f"[Native] Native Track Width: {track_width:.4f} units")
    else:
        print("[Native] ERR: Could not locate all 4 joints.")

    simulation_app.close()

if __name__ == "__main__":
    main()
