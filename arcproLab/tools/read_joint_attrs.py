# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Read joint attributes from USD.")
parser.add_argument("usd_path", type=str, help="Path to the source USD file.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

from pxr import Usd, UsdPhysics, UsdGeom

def read_joint_attrs(usd_path):
    print(f"Opening stage: {usd_path}")
    stage = Usd.Stage.Open(usd_path)
    
    def traverse(prim):
        type_name = prim.GetTypeName()
        if "Joint" in type_name:
            print(f"\nJoint: {prim.GetPath()} ({type_name})")
            
            # Check for Drive APIs
            for schema in prim.GetAppliedSchemas():
                if "DriveAPI" in schema:
                    instance_name = schema.split(":")[-1] if ":" in schema else "angular"
                    drive = UsdPhysics.DriveAPI(prim, instance_name)
                    print(f"  Drive: {schema}")
                    try:
                        print(f"    Type: {drive.GetTypeAttr().Get()}")
                        print(f"    Stiffness: {drive.GetStiffnessAttr().Get()}")
                        print(f"    Damping: {drive.GetDampingAttr().Get()}")
                        print(f"    Max Force: {drive.GetMaxForceAttr().Get()}")
                    except:
                        pass
            
            # Check for Revolute attributes
            if "Revolute" in type_name:
                rev = UsdPhysics.RevoluteJoint(prim)
                print(f"  Axis: {rev.GetAxisAttr().Get()}")
                print(f"  Lower Limit: {rev.GetLowerLimitAttr().Get()}")
                print(f"  Upper Limit: {rev.GetUpperLimitAttr().Get()}")

        for child in prim.GetChildren():
            traverse(child)

    traverse(stage.GetPseudoRoot())

def main():
    read_joint_attrs(args_cli.usd_path)
    simulation_app.close()

if __name__ == "__main__":
    main()
