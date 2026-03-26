# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Strip all collisions except wheels.")
parser.add_argument("usd_path", type=str, help="Path to the source USD file.")
parser.add_argument("output_path", type=str, help="Path to the output USD file.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

from pxr import Usd, UsdPhysics, UsdGeom

def strip_collisions(usd_path, output_path):
    print(f"Opening stage: {usd_path}")
    stage = Usd.Stage.Open(usd_path)
    
    for prim in stage.Traverse():
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            name = prim.GetName().lower()
            if "wheel" in name:
                print(f"Keeping wheel collision: {prim.GetPath()}")
            else:
                print(f"Removing non-wheel collision: {prim.GetPath()}")
                prim.RemoveAPI(UsdPhysics.CollisionAPI)
                # Also remove mesh collision if present
                if prim.HasAPI(UsdPhysics.MeshCollisionAPI):
                    prim.RemoveAPI(UsdPhysics.MeshCollisionAPI)

    stage.GetRootLayer().Export(output_path)
    print(f"Stripped robot saved to: {output_path}")

def main():
    strip_collisions(args_cli.usd_path, args_cli.output_path)
    simulation_app.close()

if __name__ == "__main__":
    main()
