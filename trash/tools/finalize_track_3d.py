# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Finalize 3D track: scale and apply robust SDF collisions.")
parser.add_argument("usd_path", type=str, help="Path to the source USD file.")
parser.add_argument("output_path", type=str, help="Path to the output USD file.")
parser.add_argument("--factor", type=float, default=2.0, help="Scaling factor.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

from pxr import Usd, UsdGeom, Gf, UsdPhysics, PhysxSchema

def finalize_track_3d(usd_path, output_path, factor):
    print(f"Loading stage: {usd_path}")
    stage = Usd.Stage.Open(usd_path)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    
    for prim in stage.Traverse():
        # 1. Scaling Logic (Transforms and Translations)
        if prim.IsA(UsdGeom.Xformable):
            xformable = UsdGeom.Xformable(prim)
            
            # Handle Scale Op
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

            # Handle Translate Op
            for op in xformable.GetOrderedXformOps():
                if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                    val = op.Get()
                    if val: op.Set(val * factor)

        # 2. Robust 3D Collision Logic (SDF)
        if prim.IsA(UsdGeom.Mesh):
            # Ensure any old collision API is replaced
            if prim.HasAPI(UsdPhysics.CollisionAPI):
                prim.RemoveAPI(UsdPhysics.CollisionAPI)
            
            # Apply standard Collision
            UsdPhysics.CollisionAPI.Apply(prim)
            
            # Apply PhysX Mesh Collision with SDF Approximation
            # SDF is required for stable navigation on thin/complex 3D meshes without a ground plane.
            mesh_coll = UsdPhysics.MeshCollisionAPI.Apply(prim)
            mesh_coll.GetApproximationAttr().Set("sdf")
            
            # Apply PhysX-specific SDF settings for maximum resolution
            physx_sdf = PhysxSchema.PhysxSDFMeshCollisionAPI.Apply(prim)
            physx_sdf.GetSdfResolutionAttr().Set(256)

    print(f"Exporting finalized asset to: {output_path}")
    stage.GetRootLayer().Export(output_path)

def main():
    finalize_track_3d(args_cli.usd_path, args_cli.output_path, args_cli.factor)
    simulation_app.close()

if __name__ == "__main__":
    main()
