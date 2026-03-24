# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Fix robot physics and joint types.")
parser.add_argument("usd_path", type=str, help="Path to the source USD file.")
parser.add_argument("output_path", type=str, help="Path to the output USD file.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

from pxr import Usd, UsdPhysics, UsdGeom, PhysxSchema

def fix_robot(usd_path, output_path):
    print(f"Opening stage: {usd_path}")
    stage = Usd.Stage.Open(usd_path)
    
    for prim in stage.Traverse():
        name = prim.GetName()
        if "Wheel__" in name or "Knuckle__" in name:
            print(f"Fixing joint: {prim.GetPath()}")
            
            # Ensure it is a RevoluteJoint (standard UsdPhysics)
            if prim.GetTypeName() not in ["RevoluteJoint", "PhysicsRevoluteJoint"]:
                print(f"  Changing type from {prim.GetTypeName()} to RevoluteJoint")
                prim.SetTypeName("RevoluteJoint")
            
            # Re-apply RevoluteJoint API
            if "Wheel" in name:
                rev = UsdPhysics.RevoluteJoint(prim)
                rev.GetAxisAttr().Set("Y")
                rev.GetLowerLimitAttr().Clear()
                rev.GetUpperLimitAttr().Clear()
                
                # Apply Drive
                drive = UsdPhysics.DriveAPI.Apply(prim, "angular")
                drive.GetTypeAttr().Set("velocity")
                drive.GetStiffnessAttr().Set(0.0)
                drive.GetDampingAttr().Set(10.0)
                drive.GetMaxForceAttr().Set(1000.0)
            else:
                rev = UsdPhysics.RevoluteJoint(prim)
                rev.GetAxisAttr().Set("Z")
                rev.GetLowerLimitAttr().Set(-30.0)
                rev.GetUpperLimitAttr().Set(30.0)
                
                # Apply Drive
                drive = UsdPhysics.DriveAPI.Apply(prim, "angular")
                drive.GetTypeAttr().Set("position")
                drive.GetStiffnessAttr().Set(1000.0)
                drive.GetDampingAttr().Set(10.0)
                drive.GetMaxForceAttr().Set(100.0)

    stage.GetRootLayer().Export(output_path)
    print(f"Fixed robot saved to: {output_path}")

def main():
    fix_robot(args_cli.usd_path, args_cli.output_path)
    simulation_app.close()

if __name__ == "__main__":
    main()
