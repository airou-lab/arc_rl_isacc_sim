# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Measure native wheelbase.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

from pxr import Usd, UsdGeom, Gf
import numpy as np

def main():
    usd_path = "f1tenth_trainer/assets/F1Tenth_Modern.usd"
    stage = Usd.Stage.Open(usd_path)
    
    # Wheel joints/links
    fl_path = "/Full_Car/Joints/Wheel__Knuckle__Front_Left"
    rl_path = "/Full_Car/Joints/Chassis__Arm_Rear_Upper_Left" # Approximation for axle center
    
    # Let's just find all wheels
    wheels = {}
    for prim in stage.Traverse():
        if "Wheel_" in prim.GetName() and prim.IsA(UsdGeom.Xformable):
            xform = UsdGeom.Xformable(prim)
            local_to_world = xform.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
            translation = local_to_world.ExtractTranslation()
            wheels[prim.GetName()] = np.array(translation)
            
    print("\n[Native] Detected Wheels:")
    for name, pos in wheels.items():
        print(f" - {name}: {pos}")
        
    # Calculate wheelbase from front vs rear wheels
    # Usually: Wheel_Front_Left, Wheel_Front_Right, Wheel_Rear_Left, Wheel_Rear_Right
    if len(wheels) >= 4:
        # X is usually the forward axis in these USDs
        # Sort wheels by X position
        sorted_wheels = sorted(wheels.items(), key=lambda x: x[1][0])
        rear_wheels = sorted_wheels[:2]
        front_wheels = sorted_wheels[-2:]
        
        rear_axle_x = (rear_wheels[0][1][0] + rear_wheels[1][1][0]) / 2.0
        front_axle_x = (front_wheels[0][1][0] + front_wheels[1][1][0]) / 2.0
        
        wheelbase = front_axle_x - rear_axle_x
        print(f"\n[Native] Native Wheelbase: {wheelbase:.4f} units")
    else:
        print("[Native] ERR: Could not identify front/rear axles.")

    simulation_app.close()

if __name__ == "__main__":
    main()
