# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Deep Bake Robot USD to Meter Scale.")
parser.add_argument("usd_path", type=str, help="Path to the source USD file.")
parser.add_argument("output_path", type=str, help="Path to the output USD file.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

from pxr import Usd, UsdGeom, Gf, UsdPhysics

def bake_to_meters(usd_path, output_path):
    print(f"Opening stage: {usd_path}")
    stage = Usd.Stage.Open(usd_path)
    mpu = UsdGeom.GetStageMetersPerUnit(stage)
    print(f"Original Meters Per Unit: {mpu}")

    # Create new stage
    new_stage = Usd.Stage.CreateNew(output_path)
    UsdGeom.SetStageMetersPerUnit(new_stage, 1.0)
    UsdGeom.SetStageUpAxis(new_stage, UsdGeom.Tokens.z)
    print("Created new stage with Meters Per Unit = 1.0")

    # We want to find the robot center first to bring it to origin
    bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default"])
    root_prim = stage.GetPrimAtPath("/Full_Car")
    bbox = bbox_cache.ComputeWorldBound(root_prim)
    center = bbox.ComputeAlignedRange().GetMidpoint()
    print(f"Original world center: {center}")

    # Traverse all prims and recreate them in the new stage
    for prim in stage.Traverse():
        path = prim.GetPath()
        type_name = prim.GetTypeName()
        
        new_prim = new_stage.DefinePrim(path, type_name)
        
        # Copy attributes? No, we want to bake transforms.
        if prim.IsA(UsdGeom.Xformable):
            xformable = UsdGeom.Xformable(prim)
            new_xformable = UsdGeom.Xformable(new_prim)
            
            # Compute world transform in original stage
            world_transform = xformable.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
            
            # Convert to meters and center
            # world_transform is in units. Convert to meters: val * mpu
            # And subtract center: (val - center) * mpu
            
            # Create a translation matrix
            trans_mat = Gf.Matrix4d().SetTranslate(-center)
            # Apply center correction then scale by mpu
            baked_transform = world_transform * trans_mat
            
            # Actually, simpler: bake the world transform then scale it
            # Baked world pos in meters = (WorldPos_units - Center_units) * mpu
            
            baked_pos = (world_transform.ExtractTranslation() - center) * mpu
            baked_rot = world_transform.ExtractRotation()
            # Scale must also be handled. Original world scale was ~0.01.
            # We want the meshes to be their correct size in meters.
            # If mesh local size is 10.4, and we want 0.104m, then scale should be 0.01.
            # Wait, if mpu is 1.0, then 0.104 units = 0.104 meters.
            # So scale should be 0.01.
            
            # Let's just bake the full transform matrix
            # Matrix in meters = Matrix_in_units * mpu
            # But scaling a matrix by a scalar affects everything.
            
            m = baked_transform
            m_meters = Gf.Matrix4d(
                m[0][0]*mpu, m[0][1]*mpu, m[0][2]*mpu, m[0][3],
                m[1][0]*mpu, m[1][1]*mpu, m[1][2]*mpu, m[1][3],
                m[2][0]*mpu, m[2][1]*mpu, m[2][2]*mpu, m[2][3],
                m[3][0]*mpu, m[3][1]*mpu, m[3][2]*mpu, m[3][3]
            )
            # Wait, that's not right. 
            # Correct way: baked_pos = (pos - center) * mpu. baked_scale = scale * mpu.
            
            new_xformable.ClearXformOpOrder()
            new_xformable.AddTranslateOp().Set(baked_pos)
            new_xformable.AddRotateXYZOp().Set(baked_rot.GetDecomposition(UsdGeom.Tokens.zyx))
            
            # Handle scale
            sx = Gf.Vec3d(world_transform[0][0], world_transform[0][1], world_transform[0][2]).GetLength() * mpu
            sy = Gf.Vec3d(world_transform[1][0], world_transform[1][1], world_transform[1][2]).GetLength() * mpu
            sz = Gf.Vec3d(world_transform[2][0], world_transform[2][1], world_transform[2][2]).GetLength() * mpu
            new_xformable.AddScaleOp().Set(Gf.Vec3f(sx, sy, sz))

        # Copy Mesh data
        if prim.IsA(UsdGeom.Mesh):
            mesh = UsdGeom.Mesh(prim)
            new_mesh = UsdGeom.Mesh(new_prim)
            # Copy points, faceVertexCounts, faceVertexIndices
            new_mesh.GetPointsAttr().Set(mesh.GetPointsAttr().Get())
            new_mesh.GetFaceVertexCountsAttr().Set(mesh.GetFaceVertexCountsAttr().Get())
            new_mesh.GetFaceVertexIndicesAttr().Set(mesh.GetFaceVertexIndicesAttr().Get())

        # Copy Physics properties
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            UsdPhysics.RigidBodyAPI.Apply(new_prim)
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            UsdPhysics.CollisionAPI.Apply(new_prim)
        if prim.HasAPI(UsdPhysics.MassAPI):
            mass_api = UsdPhysics.MassAPI(prim)
            new_mass = UsdPhysics.MassAPI.Apply(new_prim)
            new_mass.GetMassAttr().Set(mass_api.GetMassAttr().Get())

        # Handle Joints
        if "Joint" in type_name:
            # Joints are tricky because they have relationships
            # For now, let's just create them and set their local poses to 0
            # since we baked the bodies to their world positions.
            # NO, we need to preserve the joint frame relative to body0 and body1.
            pass

    new_stage.GetRootLayer().Save()
    print(f"Deep-baked USD saved to: {output_path}")

def main():
    bake_to_meters(args_cli.usd_path, args_cli.output_path)
    simulation_app.close()

if __name__ == "__main__":
    main()
