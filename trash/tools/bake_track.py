# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Flatten and Bake Track to Meters.")
parser.add_argument("usd_path", type=str, help="Path to the source USD file.")
parser.add_argument("output_path", type=str, help="Path to the output USD file.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

from pxr import Usd, UsdPhysics, UsdGeom, Gf

def bake_track(usd_path, output_path):
    print(f"Opening stage: {usd_path}")
    stage = Usd.Stage.Open(usd_path)
    mpu = UsdGeom.GetStageMetersPerUnit(stage)
    
    # 1. Create a fresh Meters-based stage
    new_stage = Usd.Stage.CreateNew(output_path)
    UsdGeom.SetStageMetersPerUnit(new_stage, 1.0)
    UsdGeom.SetStageUpAxis(new_stage, UsdGeom.Tokens.z)
    
    # Create a default root prim
    root_prim = UsdGeom.Xform.Define(new_stage, "/Track")
    new_stage.SetDefaultPrim(root_prim.GetPrim())
    print(f"Set defaultPrim to {root_prim.GetPath()}")

    # 2. Traverse original stage and bake world transforms
    for prim in stage.Traverse():
        if prim.IsA(UsdGeom.Mesh):
            name = prim.GetName().lower()
            if "terrain" in name or "road" in name or "piece" in name:
                # full path to ensure uniqueness
                rel_path = prim.GetPath().pathString.replace("/", "_")
                new_path = f"/Track/{rel_path}"
                
                new_mesh = UsdGeom.Mesh.Define(new_stage, new_path)
                
                # Get world transform
                xformable = UsdGeom.Xformable(prim)
                world_transform = xformable.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
                
                # Bake vertices
                points = prim.GetAttribute("points").Get()
                if points:
                    baked_points = [world_transform.Transform(Gf.Vec3d(p)) * mpu for p in points]
                    new_mesh.GetPointsAttr().Set(baked_points)
                
                # Copy topology
                new_mesh.GetFaceVertexCountsAttr().Set(prim.GetAttribute("faceVertexCounts").Get())
                new_mesh.GetFaceVertexIndicesAttr().Set(prim.GetAttribute("faceVertexIndices").Get())
                
                # Add collisions
                UsdPhysics.CollisionAPI.Apply(new_mesh.GetPrim())
                mesh_coll = UsdPhysics.MeshCollisionAPI.Apply(new_mesh.GetPrim())
                mesh_coll.GetApproximationAttr().Set("none")
                print(f"Baked and collidified: {new_path}")

    new_stage.GetRootLayer().Save()
    print(f"Baked Track saved to: {output_path}")

def main():
    bake_track(args_cli.usd_path, args_cli.output_path)
    simulation_app.close()

if __name__ == "__main__":
    main()
