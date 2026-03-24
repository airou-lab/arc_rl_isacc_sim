# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Scrub all physics from Robot USD and re-apply baseline.")
parser.add_argument("usd_path", type=str, help="Path to the source USD file.")
parser.add_argument("output_path", type=str, help="Path to the output USD file.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

from pxr import Usd, UsdPhysics, UsdGeom, PhysxSchema, Gf

def scrub_physics(usd_path, output_path):
    print(f"Opening stage: {usd_path}")
    stage = Usd.Stage.Open(usd_path)
    
    for prim in stage.Traverse():
        # 1. Remove ALL Physics related APIs first
        if prim.HasAPI(UsdPhysics.DriveAPI):
            # DriveAPI is a multiple-apply API, we need to find all instances
            # But we can just remove all children that are DriveAPI
            pass 
        
        # A more reliable way: iterate all schemas and remove ones we don't want
        prim.RemoveAPI(UsdPhysics.DriveAPI, "angular")
        prim.RemoveAPI(UsdPhysics.DriveAPI, "linear")
        prim.RemoveAPI(UsdPhysics.DriveAPI, "steering")
        prim.RemoveAPI(UsdPhysics.DriveAPI, "drive")
        
        # 2. Clear all attributes on Joints
        if prim.IsA(UsdPhysics.Joint):
            print(f"Scrubbing joint: {prim.GetPath()}")
            # Clear limits
            if prim.IsA(UsdPhysics.RevoluteJoint):
                rev = UsdPhysics.RevoluteJoint(prim)
                rev.GetLowerLimitAttr().Clear()
                rev.GetUpperLimitAttr().Clear()
            
            # Re-apply baseline drives ONLY to drive wheels
            name = prim.GetName()
            if "Wheel" in name:
                print(f"  Re-applying wheel drive: {name}")
                drive = UsdPhysics.DriveAPI.Apply(prim, "angular")
                drive.GetStiffnessAttr().Set(0.0)
                drive.GetDampingAttr().Set(1.0)
                drive.GetMaxForceAttr().Set(1000.0)
            elif "Knuckle" in name:
                print(f"  Re-applying steering drive: {name}")
                drive = UsdPhysics.DriveAPI.Apply(prim, "angular")
                drive.GetStiffnessAttr().Set(1000.0)
                drive.GetDampingAttr().Set(100.0)
                drive.GetMaxForceAttr().Set(100.0)

        # 3. Fix Collision contact offsets
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            px_coll = PhysxSchema.PhysxCollisionAPI.Apply(prim)
            px_coll.GetContactOffsetAttr().Set(0.02) # 2cm offset
            px_coll.GetRestOffsetAttr().Set(0.0)

    # Save
    stage.GetRootLayer().Export(output_path)
    print(f"Scrubbed and re-applied physics saved to: {output_path}")

def main():
    scrub_physics(args_cli.usd_path, args_cli.output_path)
    simulation_app.close()

if __name__ == "__main__":
    main()
