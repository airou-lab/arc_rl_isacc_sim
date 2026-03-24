# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Surgically extract and harden the road surface for 3D navigation.")
parser.add_argument("usd_path", type=str, help="Path to the source USD file.")
parser.add_argument("output_path", type=str, help="Path to save the road collision asset.")
parser.add_argument("--factor", type=float, default=2.0, help="Scaling factor.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import os
from pxr import Usd, UsdGeom, Gf, UsdPhysics, PhysxSchema

def extract_road_surface(usd_path, output_path, factor):
    print(f"Extracting road from: {usd_path}")
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
    
    # We only want the actual road surfaces
    include_keywords = ["road", "pavement", "asphalt", "marker", "yellow", "white"]

    for prim in source_stage.Traverse():
        if prim.IsA(UsdGeom.Mesh):
            path = prim.GetPath().pathString.lower()
            if any(k in path for k in include_keywords):
                mesh = UsdGeom.Mesh(prim)
                xformable = UsdGeom.Xformable(prim)
                world_transform = xformable.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
                
                points = mesh.GetPointsAttr().Get()
                if not points: continue
                
                # Transform and scale
                for p in points:
                    wp = world_transform.Transform(Gf.Vec3d(p)) * factor
                    all_points.append(Gf.Vec3f(wp))
                
                all_face_vertex_counts.extend(mesh.GetFaceVertexCountsAttr().Get())
                for i in mesh.GetFaceVertexIndicesAttr().Get():
                    all_face_vertex_indices.append(i + vertex_offset)
                
                vertex_offset += len(points)

    if not all_points:
        print("ERROR: No road meshes found!")
        return

    # 2. Create the unified Road Collision Mesh
    road_mesh = UsdGeom.Mesh.Define(new_stage, "/Track/RoadCollision")
    road_mesh.GetPointsAttr().Set(all_points)
    road_mesh.GetFaceVertexCountsAttr().Set(all_face_vertex_counts)
    road_mesh.GetFaceVertexIndicesAttr().Set(all_face_vertex_indices)
    
    UsdPhysics.CollisionAPI.Apply(road_mesh.GetPrim())
    mesh_coll = UsdPhysics.MeshCollisionAPI.Apply(road_mesh.GetPrim())
    mesh_coll.GetApproximationAttr().Set("sdf") 
    
    # Set moderate SDF resolution for stability
    physx_sdf = PhysxSchema.PhysxSDFMeshCollisionAPI.Apply(road_mesh.GetPrim())
    physx_sdf.GetSdfResolutionAttr().Set(128)
    
    # Make it invisible
    UsdGeom.Imageable(road_mesh).MakeInvisible()

    # 3. Reference the FULL city visuals
    visual_node = UsdGeom.Xform.Define(new_stage, "/Track/Visuals")
    visual_ref = visual_node.GetPrim().GetReferences()
    abs_visual_source = os.path.abspath("openStreetUSD/street_sim_repaired.usd")
    visual_ref.AddReference(abs_visual_source)
    
    # Scale robustly
    root_xform = UsdGeom.Xformable(visual_node)
    scale_op = None
    for op in root_xform.GetOrderedXformOps():
        if op.GetOpType() == UsdGeom.XformOp.TypeScale:
            scale_op = op
            break
    if not scale_op:
        scale_op = root_xform.AddScaleOp()
    scale_op.Set(Gf.Vec3f(factor))

    new_stage.GetRootLayer().Save()
    print(f"Surgical 3D Map (SDF Physics) saved to: {output_path}")

def main():
    extract_road_surface(args_cli.usd_path, args_cli.output_path, args_cli.factor)
    simulation_app.close()

if __name__ == "__main__":
    main()
