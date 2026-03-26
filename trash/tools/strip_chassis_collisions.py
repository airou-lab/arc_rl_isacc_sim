# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Strip Chassis Collisions from Robot USD.")
parser.add_argument("usd_path", type=str, help="Path to the source USD file.")
parser.add_argument("output_path", type=str, help="Path to the output USD file.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

from pxr import Usd, UsdPhysics, UsdGeom

def strip_chassis_collisions(usd_path, output_path):
    print(f"Opening stage: {usd_path}")
    stage = Usd.Stage.Open(usd_path)
    
    for prim in stage.Traverse():
        path = prim.GetPath().pathString
        # Identify chassis-related collision prims
        if "Chassis" in path and prim.HasAPI(UsdPhysics.CollisionAPI):
            print(f"Removing CollisionAPI from: {path}")
            prim.RemoveAPI(UsdPhysics.CollisionAPI)
            # Also remove PhysxCollisionAPI if present
            from pxr import PhysxSchema
            if prim.HasAPI(PhysxSchema.PhysxCollisionAPI):
                prim.RemoveAPI(PhysxSchema.PhysxCollisionAPI)

    # Save as new file
    stage.GetRootLayer().Export(output_path)
    print(f"Fixed USD saved to: {output_path}")

def main():
    strip_chassis_collisions(args_cli.usd_path, args_cli.output_path)
    simulation_app.close()

if __name__ == "__main__":
    main()
