# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Sanitize Robot USD: Flatten and Centroid-Shift.")
parser.add_argument("usd_path", type=str, help="Path to the source USD file.")
parser.add_argument("output_path", type=str, help="Path to the output USD file.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

from pxr import Usd, UsdGeom, Gf, UsdPhysics

def sanitize_v2(usd_path, output_path):
    print(f"Opening stage: {usd_path}")
    stage = Usd.Stage.Open(usd_path)
    
    # 1. FLATTEN
    flat_layer = stage.Flatten()
    flat_stage = Usd.Stage.CreateInMemory()
    flat_stage.GetRootLayer().transferContent = flat_layer
    
    # Force MetersPerUnit to 1.0 for the new stage
    UsdGeom.SetStageMetersPerUnit(flat_stage, 1.0)
    UsdGeom.SetStageUpAxis(flat_stage, UsdGeom.Tokens.z)
    
    # 2. Iterate over all Prims with RigidBodyAPI
    for prim in flat_stage.Traverse():
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            print(f"Centroid-shifting RigidBody: {prim.GetPath()}")
            
            # Compute visual center of all children meshes
            bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default"])
            bbox = bbox_cache.ComputeLocalBound(prim, Usd.TimeCode.Default(), "default")
            center_units = bbox.ComputeAlignedRange().GetMidpoint()
            print(f"  Local Center (units): {center_units}")
            
            # Convert center to meters (assuming original was 0.01 MPU)
            # Actually, the flattened stage already has the cumulative transforms.
            # If the meshes were 100x too big in raw units, we need to handle that.
            
            # Move the prim's origin to its visual center
            xform = UsdGeom.Xformable(prim)
            old_transform = xform.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
            
            # Move vertices of all children meshes to be relative to the new origin
            for child in prim.GetChildren():
                if child.IsA(UsdGeom.Mesh):
                    mesh = UsdGeom.Mesh(child)
                    points = mesh.GetPointsAttr().Get()
                    if points:
                        # Original points were in local space of the RigidBody.
                        # We want them to be in the NEW local space (offset by center).
                        new_points = [p - center_units for p in points]
                        mesh.GetPointsAttr().Set(new_points)
                        print(f"    Shifted mesh vertices: {child.GetPath()}")
            
            # Update the prim's translation to include this center shift
            # WorldPos = OldWorldPos + WorldRot * center
            # Actually, in USD, just adding a translate op works if we clear others.
            xform.ClearXformOpOrder()
            # World translation in meters
            # Original center was in units (e.g. 17). At 0.01 MPU that is 0.17m.
            world_pos_meters = old_transform.Transform(center_units) * 0.01
            xform.AddTranslateOp().Set(world_pos_meters)
            
            # Handle orientation
            rot = old_transform.ExtractRotationQuat()
            xform.AddOrientOp().Set(rot)
            
            # Scale meshes to meters (0.01)
            for child in prim.GetChildren():
                if child.IsA(UsdGeom.Mesh):
                    cxform = UsdGeom.Xformable(child)
                    cxform.ClearXformOpOrder()
                    cxform.AddScaleOp().Set(Gf.Vec3f(0.01, 0.01, 0.01))

    # 3. Handle Joints - Move to world positions (simplified)
    # This is still the hardest part. Let's try to just zero them for now.
    for prim in flat_stage.Traverse():
        if prim.IsA(UsdPhysics.Joint):
            joint = UsdPhysics.Joint(prim)
            joint.GetLocalPos0Attr().Set(Gf.Vec3f(0,0,0))
            joint.GetLocalPos1Attr().Set(Gf.Vec3f(0,0,0))

    flat_stage.GetRootLayer().Export(output_path)
    print(f"Sanitized V2 USD saved to: {output_path}")

def main():
    sanitize_v2(args_cli.usd_path, args_cli.output_path)
    simulation_app.close()

if __name__ == "__main__":
    main()
