# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Flatten and Bake Track to a Flat Plane.")
parser.add_argument("usd_path", type=str, help="Path to the source USD file.")
parser.add_argument("output_path", type=str, help="Path to the output USD file.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

from pxr import Usd, UsdPhysics, UsdGeom, Gf

def flatten_track_geom(usd_path, output_path):
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

    # 2. Traverse and flatten
    for prim in stage.Traverse():
        if prim.IsA(UsdGeom.Mesh):
            name = prim.GetName().lower()
            if "terrain" in name or "road" in name:
                rel_path = prim.GetPath().pathString.replace("/", "_")
                new_path = f"/Track/{rel_path}"
                new_mesh = UsdGeom.Mesh.Define(new_stage, new_path)
                
                xformable = UsdGeom.Xformable(prim)
                world_transform = xformable.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
                
                points = prim.GetAttribute("points").Get()
                if points:
                    # FORCE Z to 0.0 during baking
                    baked_points = []
                    for p in points:
                        wp = world_transform.Transform(Gf.Vec3d(p)) * mpu
                        baked_points.append(Gf.Vec3f(wp[0], wp[1], 0.0))
                    new_mesh.GetPointsAttr().Set(baked_points)
                
                new_mesh.GetFaceVertexCountsAttr().Set(prim.GetAttribute("faceVertexCounts").Get())
                new_mesh.GetFaceVertexIndicesAttr().Set(prim.GetAttribute("faceVertexIndices").Get())
                
                # Add collisions
                UsdPhysics.CollisionAPI.Apply(new_mesh.GetPrim())
                mesh_coll = UsdPhysics.MeshCollisionAPI.Apply(new_mesh.GetPrim())
                mesh_coll.GetApproximationAttr().Set("none")
                print(f"Flatten-baked: {new_path}")

    new_stage.GetRootLayer().Save()
    print(f"Flat Baked Track saved to: {output_path}")

def main():
    flatten_track_geom(args_cli.usd_path, args_cli.output_path)
    simulation_app.close()

if __name__ == "__main__":
    main()
