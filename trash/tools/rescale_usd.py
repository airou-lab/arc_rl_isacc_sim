# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Deep rescale USD robot.")
parser.add_argument("usd_path", type=str, help="Path to the source USD file.")
parser.add_argument("output_path", type=str, help="Path to the output USD file.")
parser.add_argument("--scale", type=float, default=0.317, help="Scale factor.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

from pxr import Usd, UsdGeom, Gf

def deep_rescale(usd_path, output_path, scale):
    print(f"Opening stage: {usd_path}")
    stage = Usd.Stage.Open(usd_path)
    
    # We want to scale EVERYTHING relative to world origin
    # The best way is to scale the translation of top-level prims AND their scale
    for prim in stage.GetPseudoRoot().GetChildren():
        if prim.IsA(UsdGeom.Xformable):
            xform = UsdGeom.Xformable(prim)
            
            # Scale translation
            for op in xform.GetOrderedXformOps():
                if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                    val = op.Get()
                    if val:
                        op.Set(Gf.Vec3d(val[0]*scale, val[1]*scale, val[2]*scale))
                elif op.GetOpType() == UsdGeom.XformOp.TypeScale:
                    val = op.Get()
                    if val:
                        op.Set(Gf.Vec3f(val[0]*scale, val[1]*scale, val[2]*scale))
            
            # If no scale op, add one
            if not any(op.GetOpType() == UsdGeom.XformOp.TypeScale for op in xform.GetOrderedXformOps()):
                scale_op = xform.AddScaleOp()
                scale_op.Set(Gf.Vec3f(scale, scale, scale))

    stage.GetRootLayer().Export(output_path)
    print(f"Deep-rescaled USD saved to: {output_path}")

def main():
    deep_rescale(args_cli.usd_path, args_cli.output_path, args_cli.scale)
    simulation_app.close()

if __name__ == "__main__":
    main()
