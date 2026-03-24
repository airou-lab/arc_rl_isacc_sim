# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Zero the robot height.")
parser.add_argument("usd_path", type=str, help="Path to the source USD file.")
parser.add_argument("output_path", type=str, help="Path to the output USD file.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

from pxr import Usd, UsdGeom, Gf

def zero_robot(usd_path, output_path):
    print(f"Opening stage: {usd_path}")
    stage = Usd.Stage.Open(usd_path)
    
    # 1. Compute BBox of WHEELS specifically
    # Because we want the wheels to be on the ground
    bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default"])
    
    min_z = 999999
    found_wheels = False
    for prim in stage.TraverseAll():
        if "Wheel" in prim.GetName() and prim.IsA(UsdGeom.Mesh):
            bbox = bbox_cache.ComputeWorldBound(prim)
            range = bbox.ComputeAlignedRange()
            low_z = range.GetMin()[2]
            if low_z < min_z:
                min_z = low_z
            found_wheels = True
            print(f"Wheel {prim.GetPath()} min Z: {low_z}")

    if not found_wheels:
        print("No wheels found! Using global BBox.")
        bbox = bbox_cache.ComputeWorldBound(stage.GetPseudoRoot())
        range = bbox.ComputeAlignedRange()
        min_z = range.GetMin()[2]

    print(f"Lowest point found: {min_z}")

    # 2. Subtract THIS min_z from all top-level prims (X and Y too if they are huge)
    # Actually let's just center XY and zero Z.
    bbox = bbox_cache.ComputeWorldBound(stage.GetPseudoRoot())
    range = bbox.ComputeAlignedRange()
    center = range.GetMidpoint()
    
    # Offset vector: move XY center to 0, and min_z to 0
    offset = Gf.Vec3d(center[0], center[1], min_z)
    print(f"Total offset to apply: {offset}")

    for prim in stage.GetPseudoRoot().GetChildren():
        if prim.IsA(UsdGeom.Xformable):
            xform = UsdGeom.Xformable(prim)
            # Clear ops and add one clean translate
            xform.ClearXformOpOrder()
            translate_op = xform.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble)
            translate_op.Set(-offset)
            print(f"Zeroed top-level prim: {prim.GetPath()}")

    stage.GetRootLayer().Export(output_path)
    print(f"Zeroed USD saved to: {output_path}")

def main():
    zero_robot(args_cli.usd_path, args_cli.output_path)
    simulation_app.close()

if __name__ == "__main__":
    main()
