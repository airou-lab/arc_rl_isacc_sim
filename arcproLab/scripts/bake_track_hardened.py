# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Harden 3D track: merge ONLY terrain tiles into a single collision mesh.")
parser.add_argument("usd_path", type=str, help="Path to the source USD file.")
parser.add_argument("output_path", type=str, help="Path to the output USD file.")
parser.add_argument("--factor", type=float, default=2.0, help="Scaling factor.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import os
from pxr import Usd, UsdGeom, Gf, UsdPhysics

def bake_hardened_track_surgical(usd_path, output_path, factor):
    print(f"Baking surgical hardened track from: {usd_path}")
    source_stage = Usd.Stage.Open(usd_path)
    
    new_stage = Usd.Stage.CreateNew(output_path)
    UsdGeom.SetStageMetersPerUnit(new_stage, 1.0)
    UsdGeom.SetStageUpAxis(new_stage, UsdGeom.Tokens.z)
    
    root_prim = UsdGeom.Xform.Define(new_stage, "/Track")
    new_stage.SetDefaultPrim(root_prim.GetPrim())

    all_points = []
    all_face_vertex_counts = []
    all_face_vertex_indices = []
    vertex_offset = 0
    
    # Exclusion list to avoid baking the robot into the ground
    exclude_keywords = ["Robot", "Chassis", "Wheel", "Suspension", "Knuckle", "Shock", "Upright"]

    for prim in source_stage.Traverse():
        if prim.IsA(UsdGeom.Mesh):
            path = prim.GetPath().pathString
            if any(k in path for k in exclude_keywords):
                print(f"Skipping robot-related mesh: {path}")
                continue
                
            mesh = UsdGeom.Mesh(prim)
            xformable = UsdGeom.Xformable(prim)
            world_transform = xformable.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
            
            points = mesh.GetPointsAttr().Get()
            if not points: continue
            
            # Transform and scale points
            for p in points:
                wp = world_transform.Transform(Gf.Vec3d(p)) * factor
                all_points.append(Gf.Vec3f(wp))
            
            all_face_vertex_counts.extend(mesh.GetFaceVertexCountsAttr().Get())
            for i in mesh.GetFaceVertexIndicesAttr().Get():
                all_face_vertex_indices.append(i + vertex_offset)
            
            vertex_offset += len(points)
            print(f"Merged track mesh: {prim.GetName()}")

    # 2. Create the unified Collision Mesh
    collision_mesh = UsdGeom.Mesh.Define(new_stage, "/Track/CollisionMesh")
    collision_mesh.GetPointsAttr().Set(all_points)
    collision_mesh.GetFaceVertexCountsAttr().Set(all_face_vertex_counts)
    collision_mesh.GetFaceVertexIndicesAttr().Set(all_face_vertex_indices)
    
    UsdPhysics.CollisionAPI.Apply(collision_mesh.GetPrim())
    mesh_coll = UsdPhysics.MeshCollisionAPI.Apply(collision_mesh.GetPrim())
    mesh_coll.GetApproximationAttr().Set("none") # Direct Triangle Mesh
    
    # 3. Add visual reference (Surgical path fix)
    visual_ref = root_prim.GetPrim().GetReferences()
    # Use absolute path for reliability
    abs_repaired_path = os.path.abspath("openStreetUSD/street_sim_repaired.usd")
    visual_ref.AddReference(abs_repaired_path)
    
    # Scale the visual reference to match the collision mesh
    root_xform = UsdGeom.Xformable(root_prim)
    scale_op = None
    for op in root_xform.GetOrderedXformOps():
        if op.GetOpType() == UsdGeom.XformOp.TypeScale:
            scale_op = op
            break
    if not scale_op:
        scale_op = root_xform.AddScaleOp()
    scale_op.Set(Gf.Vec3f(factor))

    new_stage.GetRootLayer().Save()
    print(f"Surgical Hardened 3D track saved to: {output_path}")

def main():
    bake_hardened_track_surgical(args_cli.usd_path, args_cli.output_path, args_cli.factor)
    simulation_app.close()

if __name__ == "__main__":
    main()
