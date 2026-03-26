# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Center Geometry of Robot USD.")
parser.add_argument("usd_path", type=str, help="Path to the source USD file.")
parser.add_argument("output_path", type=str, help="Path to the output USD file.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

from pxr import Usd, UsdGeom, Gf

def center_geometry(usd_path, output_path):
    print(f"Opening stage: {usd_path}")
    stage = Usd.Stage.Open(usd_path)
    
    root_prim = stage.GetPrimAtPath("/Full_Car")
    if not root_prim:
        print("Error: /Full_Car not found")
        return

    # Compute the current bounding box of the whole car
    bbox = UsdGeom.Imageable(root_prim).ComputeLocalBound(Usd.TimeCode.Default(), "default")
    box_range = bbox.GetRange()
    min_pt = box_range.GetMin()
    max_pt = box_range.GetMax()
    center = (min_pt + max_pt) / 2.0
    
    print(f"Current BBox Min: {min_pt}, Max: {max_pt}")
    print(f"Calculated Center: {center}")
    
    # We want the bottom center to be at (0,0,0) or (0,0,height)
    # Let's just center XY and set Z min to 0
    offset = Gf.Vec3d(-center[0], -center[1], -min_pt[2])
    print(f"Applying offset: {offset}")
    
    # Apply this offset to all direct children of Full_Car to keep them relative
    for child in root_prim.GetChildren():
        if child.IsA(UsdGeom.Xformable):
            xform = UsdGeom.Xformable(child)
            # Add an additional translation to move the geometry
            # Note: This is a hack, proper centering involves baking. 
            # But let's see if this fixes the spawn.
            xform.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(offset)

    stage.GetRootLayer().Export(output_path)
    print(f"Centered USD saved to: {output_path}")

def main():
    center_geometry(args_cli.usd_path, args_cli.output_path)
    simulation_app.close()

if __name__ == "__main__":
    main()
