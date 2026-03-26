# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Scale a USD asset by a factor and enable 3D collisions.")
parser.add_argument("usd_path", type=str, help="Path to the USD file to scale.")
parser.add_argument("output_path", type=str, help="Path to save the scaled USD file.")
parser.add_argument("--factor", type=float, default=100.0, help="Scaling factor.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

from pxr import Usd, UsdGeom, Gf, UsdPhysics

def scale_and_solidify(usd_path, output_path, factor):
    print(f"Scaling {usd_path} by {factor}x and enabling 3D collisions...")
    stage = Usd.Stage.Open(usd_path)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    
    for prim in stage.Traverse():
        # 1. Scaling
        if prim.IsA(UsdGeom.Xformable):
            xformable = UsdGeom.Xformable(prim)
            scale_op = None
            for op in xformable.GetOrderedXformOps():
                if op.GetOpType() == UsdGeom.XformOp.TypeScale:
                    scale_op = op
                    break
            if scale_op:
                val = scale_op.Get()
                scale_op.Set(val * factor if val else Gf.Vec3f(factor))
            else:
                xformable.AddScaleOp().Set(Gf.Vec3f(factor))

            for op in xformable.GetOrderedXformOps():
                if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                    val = op.Get()
                    if val: op.Set(val * factor)

        # 2. Physics (Triangle Mesh for static 3D navigation)
        if prim.IsA(UsdGeom.Mesh):
            UsdPhysics.CollisionAPI.Apply(prim)
            mesh_coll = UsdPhysics.MeshCollisionAPI.Apply(prim)
            # 'none' uses the raw triangle mesh for collision - best for static terrain
            mesh_coll.GetApproximationAttr().Set("none")

    stage.GetRootLayer().Export(output_path)
    print(f"3D Map asset saved to: {output_path}")

def main():
    scale_and_solidify(args_cli.usd_path, args_cli.output_path, args_cli.factor)
    simulation_app.close()

if __name__ == "__main__":
    main()
