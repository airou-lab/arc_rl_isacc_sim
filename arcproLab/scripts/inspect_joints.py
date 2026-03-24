# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Inspect USD for joint properties.")
parser.add_argument("usd_path", type=str, help="Path to the USD file.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

from pxr import Usd, UsdPhysics, UsdGeom

def main():
    print(f"Opening stage: {args_cli.usd_path}")
    stage = Usd.Stage.Open(args_cli.usd_path)
    
    print("\n--- Joint Properties ---")
    for prim in stage.Traverse():
        if prim.IsA(UsdPhysics.Joint):
            print(f"\nJoint: {prim.GetPath()}")
            # Check for drive
            for child in prim.GetChildren():
                if child.HasAPI(UsdPhysics.DriveAPI):
                    drive = UsdPhysics.DriveAPI(child)
                    print(f"  Drive: {child.GetName()}")
                    print(f"    Type: {drive.GetDriveTypeAttr().Get()}")
                    print(f"    Stiffness: {drive.GetStiffnessAttr().Get()}")
                    print(f"    Damping: {drive.GetDampingAttr().Get()}")
            
            # Check for limits
            if prim.IsA(UsdPhysics.RevoluteJoint):
                joint = UsdPhysics.RevoluteJoint(prim)
                print(f"  Revolute Limits: {joint.GetLowerLimitAttr().Get()} to {joint.GetUpperLimitAttr().Get()}")

    simulation_app.close()

if __name__ == "__main__":
    main()
