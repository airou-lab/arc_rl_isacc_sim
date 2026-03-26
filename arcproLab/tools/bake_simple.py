# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Bake robot into a simple 7-body articulation.")
parser.add_argument("usd_path", type=str, help="Path to the source USD file.")
parser.add_argument("output_path", type=str, help="Path to the output USD file.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

from pxr import Usd, UsdGeom, Gf, UsdPhysics

def bake_simple(usd_path, output_path):
    print(f"Opening stage: {usd_path}")
    stage = Usd.Stage.Open(usd_path)
    mpu = UsdGeom.GetStageMetersPerUnit(stage)
    
    # 1. Map component names to their original world positions (in meters)
    component_data = {}
    
    # We look for specific components by name
    targets = {
        "Chassis": "Chassis",
        "Wheel_Front_Left": "Wheel_Front_Left",
        "Wheel_Front_Right": "Wheel_Front_Right",
        "Wheel_Rear_Left": "Wheel_Rear_Left",
        "Wheel_Rear_Right": "Wheel_Rear_Right",
        "Steering_Knuckle_Left": "Steering_Knuckle_Left",
        "Steering_Knuckle_Right": "Steering_Knuckle_Right"
    }

    bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default"])

    for prim in stage.Traverse():
        name = prim.GetName()
        for key, target_name in targets.items():
            if name == target_name:
                xformable = UsdGeom.Xformable(prim)
                world_transform = xformable.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
                
                # Compute centroid of vertices if it's a mesh, or just use pivot
                local_center = Gf.Vec3d(0)
                if prim.IsA(UsdGeom.Mesh):
                    mesh = UsdGeom.Mesh(prim)
                    points = mesh.GetPointsAttr().Get()
                    if points:
                        c = Gf.Vec3f(0)
                        for p in points: c += p
                        local_center = Gf.Vec3d(c / len(points))
                else:
                    # Look for mesh children
                    for child in prim.GetChildren():
                        if child.IsA(UsdGeom.Mesh):
                            mesh = UsdGeom.Mesh(child)
                            points = mesh.GetPointsAttr().Get()
                            if points:
                                c = Gf.Vec3f(0)
                                for p in points: c += p
                                local_center = Gf.Vec3d(c / len(points))
                                break
                
                world_pos = world_transform.Transform(local_center) * mpu
                component_data[key] = {
                    "path": prim.GetPath(),
                    "world_pos": world_pos,
                    "world_rot": world_transform.ExtractRotationQuat(),
                    "mesh_prim": prim if prim.IsA(UsdGeom.Mesh) else [c for c in prim.GetChildren() if c.IsA(UsdGeom.Mesh)][0]
                }
                print(f"Found {key} at {world_pos}")

    # 2. Create new Stage
    new_stage = Usd.Stage.CreateNew(output_path)
    UsdGeom.SetStageMetersPerUnit(new_stage, 1.0)
    UsdGeom.SetStageUpAxis(new_stage, UsdGeom.Tokens.z)
    root_prim = UsdGeom.Xform.Define(new_stage, "/Robot")
    new_stage.SetDefaultPrim(root_prim.GetPrim())
    # Apply ArticulationRoot to the root
    UsdPhysics.ArticulationRootAPI.Apply(root_prim.GetPrim())

    # 3. Define Bodies at their world positions relative to Chassis
    chassis_pos = component_data["Chassis"]["world_pos"]
    
    for key, data in component_data.items():
        rel_pos = data["world_pos"] - chassis_pos
        new_path = f"/Robot/{key}"
        new_rb = UsdGeom.Xform.Define(new_stage, new_path)
        new_rb.AddTranslateOp().Set(rel_pos)
        new_rb.AddOrientOp().Set(data["world_rot"])
        
        # Physics
        UsdPhysics.RigidBodyAPI.Apply(new_rb.GetPrim())
        mass = UsdPhysics.MassAPI.Apply(new_rb.GetPrim())
        mass.GetMassAttr().Set(4.0 if key == "Chassis" else 0.1)
        UsdPhysics.CollisionAPI.Apply(new_rb.GetPrim())
        
        # Mesh
        new_mesh_path = f"{new_path}/{key}_Mesh"
        new_mesh = UsdGeom.Mesh.Define(new_stage, new_mesh_path)
        
        # Bake vertices relative to the new RB origin
        old_mesh = data["mesh_prim"]
        points = old_mesh.GetAttribute("points").Get()
        old_mesh_world_transform = UsdGeom.Xformable(old_mesh).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        
        baked_points = []
        for p in points:
            wp = old_mesh_world_transform.Transform(Gf.Vec3d(p)) * mpu
            lp = wp - data["world_pos"]
            # Rotate back to local frame since the RB already has the world rotation
            # Actually, the RB's local frame is already rotated by 'world_rot'.
            # lp is in world-aligned metric space. We need it in the RB's local space.
            lp_local = data["world_rot"].GetInverse().Transform(lp)
            baked_points.append(Gf.Vec3f(lp_local))
        
        new_mesh.GetPointsAttr().Set(baked_points)
        new_mesh.GetFaceVertexCountsAttr().Set(old_mesh.GetAttribute("faceVertexCounts").Get())
        new_mesh.GetFaceVertexIndicesAttr().Set(old_mesh.GetAttribute("faceVertexIndices").Get())
        
        # Mesh scale correction (must be 1.0 because we baked everything to meters)
        UsdGeom.Xformable(new_mesh).AddScaleOp().Set(Gf.Vec3f(1,1,1))

    # 4. Create Joints
    def create_revolute(name, p0_key, p1_key, axis, is_drive=False, drive_type="velocity"):
        j_path = f"/Robot/{name}"
        j_prim = new_stage.DefinePrim(j_path, "PhysicsRevoluteJoint")
        joint = UsdPhysics.RevoluteJoint(j_prim)
        joint.GetBody0Rel().SetTargets([f"/Robot/{p0_key}"])
        joint.GetBody1Rel().SetTargets([f"/Robot/{p1_key}"])
        
        # Calculate local offset
        w0 = component_data[p0_key]["world_pos"]
        w1 = component_data[p1_key]["world_pos"]
        # localPos0 = offset from p0 origin to p1 origin in p0 frame
        offset = w1 - w0
        local_offset = component_data[p0_key]["world_rot"].GetInverse().Transform(offset)
        
        joint.GetLocalPos0Attr().Set(Gf.Vec3f(local_offset))
        joint.GetLocalPos1Attr().Set(Gf.Vec3f(0,0,0))
        joint.GetAxisAttr().Set(axis)
        
        if is_drive:
            drive = UsdPhysics.DriveAPI.Apply(j_prim, "angular")
            drive.GetTypeAttr().Set(drive_type)
            drive.GetStiffnessAttr().Set(1000.0 if drive_type == "position" else 0.0)
            drive.GetDampingAttr().Set(10.0)
            drive.GetMaxForceAttr().Set(1000.0)

    # Steering Joints
    create_revolute("Steer_FL", "Chassis", "Steering_Knuckle_Left", "Z", True, "position")
    create_revolute("Steer_FR", "Chassis", "Steering_Knuckle_Right", "Z", True, "position")
    
    # Drive Joints
    create_revolute("Drive_FL", "Steering_Knuckle_Left", "Wheel_Front_Left", "Y", True, "velocity")
    create_revolute("Drive_FR", "Steering_Knuckle_Right", "Wheel_Front_Right", "Y", True, "velocity")
    create_revolute("Drive_RL", "Chassis", "Wheel_Rear_Left", "Y", True, "velocity")
    create_revolute("Drive_RR", "Chassis", "Wheel_Rear_Right", "Y", True, "velocity")

    new_stage.GetRootLayer().Save()
    print(f"Simple Baked Robot saved to: {output_path}")

def main():
    bake_simple(args_cli.usd_path, args_cli.output_path)
    simulation_app.close()

if __name__ == "__main__":
    main()
