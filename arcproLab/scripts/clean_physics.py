# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Clean all physics from Robot USD.")
parser.add_argument("usd_path", type=str, help="Path to the source USD file.")
parser.add_argument("output_path", type=str, help="Path to the output USD file.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

from pxr import Usd, UsdPhysics, UsdGeom, PhysxSchema, Gf

def clean_physics(usd_path, output_path):
    print(f"Opening stage: {usd_path}")
    stage = Usd.Stage.Open(usd_path)
    
    for prim in stage.Traverse():
        # 1. Remove any old DriveAPIs
        for child in prim.GetChildren():
            if child.HasAPI(UsdPhysics.DriveAPI):
                prim.RemoveAPI(UsdPhysics.DriveAPI, child.GetName())
        
        # 2. Setup Joints for Isaac Lab Control
        if prim.IsA(UsdPhysics.Joint):
            print(f"Cleaning joint: {prim.GetPath()}")
            # Clear all physical resistance
            if prim.HasAPI(PhysxSchema.PhysxJointAPI):
                px_joint = PhysxSchema.PhysxJointAPI.Apply(prim)
                px_joint.GetJointFrictionAttr().Set(0.0)
            
            # Add an empty angular drive to ensure Isaac Lab sees it as driveable
            drive_api = UsdPhysics.DriveAPI.Apply(prim, "angular")
            drive_api.GetStiffnessAttr().Set(0.0)
            drive_api.GetDampingAttr().Set(0.0)
            drive_api.GetMaxForceAttr().Set(1000.0)

    # Save
    stage.GetRootLayer().Export(output_path)
    print(f"Cleaned USD saved to: {output_path}")

def main():
    clean_physics(args_cli.usd_path, args_cli.output_path)
    simulation_app.close()

if __name__ == "__main__":
    main()
