# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Nuclear-fix robot joints.")
parser.add_argument("usd_path", type=str, help="Path to the source USD file.")
parser.add_argument("output_path", type=str, help="Path to the output USD file.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

from pxr import Usd, UsdPhysics, UsdGeom

def nuclear_fix(usd_path, output_path):
    print(f"Opening stage: {usd_path}")
    stage = Usd.Stage.Open(usd_path)
    
    for prim in stage.TraverseAll():
        name = prim.GetName()
        is_wheel = "Wheel__" in name
        is_steer = "Knuckle__Upright" in name
        
        if is_wheel or is_steer:
            print(f"Fixing joint: {prim.GetPath()}")
            
            # Use PhysicsRevoluteJoint as it seems to be required by this environment
            prim.SetTypeName("PhysicsRevoluteJoint")
            
            rev = UsdPhysics.RevoluteJoint(prim)
            
            if is_wheel:
                rev.GetAxisAttr().Set("Y")
                rev.GetLowerLimitAttr().Clear()
                rev.GetUpperLimitAttr().Clear()
                
                drive = UsdPhysics.DriveAPI.Apply(prim, "angular")
                drive.GetTypeAttr().Set("velocity")
                drive.GetStiffnessAttr().Set(0.0)
                drive.GetDampingAttr().Set(10.0)
                drive.GetMaxForceAttr().Set(1000.0)
            else:
                rev.GetAxisAttr().Set("Z")
                rev.GetLowerLimitAttr().Set(-30.0)
                rev.GetUpperLimitAttr().Set(30.0)
                
                drive = UsdPhysics.DriveAPI.Apply(prim, "angular")
                drive.GetTypeAttr().Set("position")
                drive.GetStiffnessAttr().Set(1000.0)
                drive.GetDampingAttr().Set(10.0)
                drive.GetMaxForceAttr().Set(100.0)

    stage.GetRootLayer().Export(output_path)
    print(f"Fixed USD saved to: {output_path}")

def main():
    nuclear_fix(args_cli.usd_path, args_cli.output_path)
    simulation_app.close()

if __name__ == "__main__":
    main()
