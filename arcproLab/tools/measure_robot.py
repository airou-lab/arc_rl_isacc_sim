# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Measure Robot Dimensions.")
parser.add_argument("usd_path", type=str, help="Path to the source USD file.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

from pxr import Usd, UsdGeom, Gf

def measure_robot(usd_path):
    print(f"Opening stage: {usd_path}")
    stage = Usd.Stage.Open(usd_path)
    
    for prim in stage.Traverse():
        if "Wheel" in prim.GetName() and prim.IsA(UsdGeom.Mesh):
            bbox = UsdGeom.Imageable(prim).ComputeLocalBound(Usd.TimeCode.Default(), "default")
            box_range = bbox.GetRange()
            min_pt = box_range.GetMin()
            max_pt = box_range.GetMax()
            size = max_pt - min_pt
            print(f"\nWheel Prim: {prim.GetPath()}")
            print(f"  Local BBox Min: {min_pt}")
            print(f"  Local BBox Max: {max_pt}")
            print(f"  Calculated Diameter: {size}")
            
            # Check for parent transforms
            parent = prim.GetParent()
            xform = UsdGeom.Xformable(parent).GetLocalTransformation()
            print(f"  Parent Transform: {xform}")

def main():
    measure_robot(args_cli.usd_path)
    simulation_app.close()

if __name__ == "__main__":
    main()
