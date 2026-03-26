# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Measure wheel ground clearance.")
parser.add_argument("usd_path", type=str, help="Path to the source USD file.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

from pxr import Usd, UsdGeom, Gf

def measure_clearance(usd_path):
    print(f"Opening stage: {usd_path}")
    stage = Usd.Stage.Open(usd_path)
    
    for prim in stage.TraverseAll():
        name = prim.GetName()
        if "Wheel" in name:
            print(f"Found Prim: {prim.GetPath()} ({prim.GetTypeName()})")
            if prim.IsA(UsdGeom.Mesh):
                bbox = UsdGeom.Imageable(prim).ComputeLocalBound(Usd.TimeCode.Default(), "default")
                box_range = bbox.GetRange()
                
                xformable = UsdGeom.Xformable(prim)
                world_transform = xformable.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
                
                min_pt = world_transform.Transform(box_range.GetMin())
                max_pt = world_transform.Transform(box_range.GetMax())
                
                print(f"  World Min: {min_pt}")
                print(f"  World Max: {max_pt}")
                print(f"  Lowest Point (Z): {min_pt[2]}")

def main():
    measure_clearance(args_cli.usd_path)
    simulation_app.close()

if __name__ == "__main__":
    main()
