# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Deep center all components of the robot.")
parser.add_argument("usd_path", type=str, help="Path to the source USD file.")
parser.add_argument("output_path", type=str, help="Path to the output USD file.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

from pxr import Usd, UsdGeom, Gf, UsdPhysics

def deep_center(usd_path, output_path):
    print(f"Opening stage: {usd_path}")
    stage = Usd.Stage.Open(usd_path)
    
    # 1. Compute BBox of everything
    bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default"])
    bbox = bbox_cache.ComputeWorldBound(stage.GetPseudoRoot())
    range = bbox.ComputeAlignedRange()
    center = range.GetMidpoint()
    print(f"Computed world center: {center}")

    # 2. Iterate over all top-level prims and move them
    # We want to move EVERY top-level Xform so that the whole group is at origin
    for prim in stage.GetPseudoRoot().GetChildren():
        if prim.IsA(UsdGeom.Xformable):
            xform = UsdGeom.Xformable(prim)
            # Add a translate op at the START of the stack
            # or just clear and set if we want to be aggressive
            ops = xform.GetOrderedXformOps()
            
            # Find existing translate or add new one
            found = False
            for op in ops:
                if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                    val = op.Get()
                    if val is None: val = Gf.Vec3d(0)
                    op.Set(val - center)
                    found = True
                    break
            
            if not found:
                translate_op = xform.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble)
                translate_op.Set(-center)
            
            print(f"Moved top-level prim: {prim.GetPath()}")

    # 3. Handle Joints. Joints use world positions for their frames usually.
    # Actually, in USD Physics, joints use local Pos0 and Pos1.
    # If we moved the bodies, the local offsets should remain the same.
    # BUT, if the joints are at world positions, we might need to adjust them.
    # For now, top-level move should suffice if the hierarchy is intact.

    stage.GetRootLayer().Export(output_path)
    print(f"Deep-centered USD saved to: {output_path}")

def main():
    deep_center(args_cli.usd_path, args_cli.output_path)
    simulation_app.close()

if __name__ == "__main__":
    main()
