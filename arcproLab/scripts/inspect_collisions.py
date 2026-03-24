# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Inspect USD for collisions.")
parser.add_argument("usd_path", type=str, help="Path to the USD file.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

from pxr import Usd, UsdPhysics, UsdGeom

def main():
    print(f"Opening stage: {args_cli.usd_path}")
    stage = Usd.Stage.Open(args_cli.usd_path)
    
    print("\n--- Collision Prims ---")
    collision_count = 0
    for prim in stage.Traverse():
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            print(f"Collision: {prim.GetPath()}")
            collision_count += 1
    
    if collision_count == 0:
        print("NO COLLISIONS FOUND!")
    else:
        print(f"Total collisions: {collision_count}")

    simulation_app.close()

if __name__ == "__main__":
    main()
