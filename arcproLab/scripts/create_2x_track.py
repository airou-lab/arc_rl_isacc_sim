# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Create a 2x scaled visual track with a solid collision floor.")
parser.add_argument("source_usd", type=str, help="Path to the original textured track.")
parser.add_argument("output_usd", type=str, help="Path for the output asset.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import os
from pxr import Usd, UsdGeom, Gf, UsdPhysics

def create_hardened_2x_track(source_usd, output_usd):
    # F1Tenth Spec
    factor = 2.0
    
    # Create new stage
    stage = Usd.Stage.CreateNew(output_usd)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    
    root_prim = UsdGeom.Xform.Define(stage, "/Track")
    stage.SetDefaultPrim(root_prim.GetPrim())

    # 1. Visual reference (original textured track)
    visual_ref = root_prim.GetPrim().GetReferences()
    visual_ref.AddReference(os.path.abspath(source_usd))
    # Scale visuals to 2x
    root_xform = UsdGeom.Xformable(root_prim)
    scale_op = None
    for op in root_xform.GetOrderedXformOps():
        if op.GetOpType() == UsdGeom.XformOp.TypeScale:
            scale_op = op
            break
    if not scale_op:
        scale_op = root_xform.AddScaleOp()
    scale_op.Set(Gf.Vec3f(factor))

    # 2. Solid Collision Floor (Matches the 12.3m x 11.9m footprint at 2x)
    floor_path = "/Track/CollisionFloor"
    floor = UsdGeom.Cube.Define(stage, floor_path)
    # Dimensions: 12.3 * 2 = 24.6, 11.9 * 2 = 23.8
    # Center it and place at z=0
    floor.AddTranslateOp().Set(Gf.Vec3d(0, 0, -0.05)) # 10cm thick, top at 0
    floor.GetSizeAttr().Set(1.0)
    floor.AddScaleOp().Set(Gf.Vec3f(25.0, 25.0, 0.1))
    
    # Make it invisible but solid
    UsdGeom.Imageable(floor).MakeInvisible()
    UsdPhysics.CollisionAPI.Apply(floor.GetPrim())

    stage.GetRootLayer().Save()
    print(f"Hardened 2x Track saved to: {output_usd}")

def main():
    create_hardened_2x_track(args_cli.source_usd, args_cli.output_usd)
    simulation_app.close()

if __name__ == "__main__":
    main()
