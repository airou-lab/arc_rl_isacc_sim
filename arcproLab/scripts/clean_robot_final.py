# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Clean and Center Robot without breaking hierarchy.")
parser.add_argument("usd_path", type=str, help="Path to the source USD file.")
parser.add_argument("output_path", type=str, help="Path to the output USD file.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

from pxr import Usd, UsdGeom, Gf, UsdPhysics

def clean_robot_final(usd_path, output_path):
    print(f"Opening stage: {usd_path}")
    stage = Usd.Stage.Open(usd_path)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)

    # 1. Find the Chassis center to use as the new origin
    chassis_prim = stage.GetPrimAtPath("/Full_Car/Rigid_Bodies/Chassis")
    bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default"])
    bbox = bbox_cache.ComputeWorldBound(chassis_prim)
    chassis_center_units = bbox.ComputeAlignedRange().GetMidpoint()
    print(f"Chassis center (units): {chassis_center_units}")

    # 2. Iterate over all meshes, bake their original unit-scale (0.01) and center them relative to Chassis
    # We will move the TOP LEVEL /Full_Car to compensate.
    
    # Actually, let's just move /Full_Car to -chassis_center_units
    root = stage.GetPrimAtPath("/Full_Car")
    xform = UsdGeom.Xformable(root)
    xform.ClearXformOpOrder()
    # Move so chassis is at origin in units
    xform.AddTranslateOp().Set(-chassis_center_units)
    # And scale the whole car to meters (0.01)
    xform.AddScaleOp().Set(Gf.Vec3f(0.01, 0.01, 0.01))
    
    # 3. Fix Joints
    # Since we only moved the root and scaled it, the relative offsets *should* be fine
    # IF they were in units.
    for prim in stage.Traverse():
        if prim.IsA(UsdPhysics.RevoluteJoint):
            rev = UsdPhysics.RevoluteJoint(prim)
            rev.GetAxisAttr().Set("Y" if "Wheel" in prim.GetName() else "Z")
            # Apply Drive
            drive = UsdPhysics.DriveAPI.Apply(prim, "angular")
            if "Wheel" in prim.GetName():
                drive.GetTypeAttr().Set("velocity")
            else:
                drive.GetTypeAttr().Set("position")

    stage.GetRootLayer().Export(output_path)
    print(f"Cleaned Robot saved to: {output_path}")

def main():
    clean_robot_final(args_cli.usd_path, args_cli.output_path)
    simulation_app.close()

if __name__ == "__main__":
    main()
