# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Bake robot into a perfectly flat 7-body articulation.")
parser.add_argument("usd_path", type=str, help="Path to the source USD file.")
parser.add_argument("output_path", type=str, help="Path to the output USD file.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

from pxr import Usd, UsdGeom, Gf, UsdPhysics, PhysxSchema

def bake_flat(usd_path, output_path):
    print(f"Opening source stage: {usd_path}")
    stage = Usd.Stage.Open(usd_path)
    mpu = UsdGeom.GetStageMetersPerUnit(stage)
    
    # Target components
    targets = {
        "Chassis": "Chassis",
        "Wheel_FL": "Wheel_Front_Left",
        "Wheel_FR": "Wheel_Front_Right",
        "Wheel_RL": "Wheel_Rear_Left",
        "Wheel_RR": "Wheel_Rear_Right",
        "Knuckle_L": "Steering_Knuckle_Left",
        "Knuckle_R": "Steering_Knuckle_Right"
    }

    comp_data = {}
    
    # 1. Collect world positions and meshes
    for prim in stage.Traverse():
        name = prim.GetName()
        for key, target_name in targets.items():
            if name == target_name:
                xformable = UsdGeom.Xformable(prim)
                world_transform = xformable.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
                
                # Find mesh child
                mesh_prim = None
                for child in prim.GetChildren():
                    if child.IsA(UsdGeom.Mesh):
                        mesh_prim = child
                        break
                if not mesh_prim and prim.IsA(UsdGeom.Mesh):
                    mesh_prim = prim
                
                if mesh_prim:
                    mesh_geom = UsdGeom.Mesh(mesh_prim)
                    pts = mesh_geom.GetPointsAttr().Get()
                    centroid = Gf.Vec3f(0)
                    if pts:
                        for p in pts: centroid += p
                        centroid /= len(pts)
                    
                    world_pos = world_transform.Transform(Gf.Vec3d(centroid))
                    
                    comp_data[key] = {
                        "world_pos": world_pos,
                        "world_rot": world_transform.ExtractRotationQuat(),
                        "mesh_prim": mesh_prim
                    }
                    print(f"Found {key} at {world_pos}")

    # Heuristic Scale Check
    try:
        import torch
        wb_meters = torch.norm(torch.tensor(comp_data["Wheel_FL"]["world_pos"]) - torch.tensor(comp_data["Wheel_RL"]["world_pos"]))
        if wb_meters < 0.05: # Less than 5cm?
            print(f"Heuristic Scale Check: Car is too small ({wb_meters:.4f}m wheelbase). Scaling up 100x.")
            for key in comp_data:
                comp_data[key]["world_pos"] *= 100.0
    except: pass

    # 2. Create Flat Stage
    new_stage = Usd.Stage.CreateNew(output_path)
    UsdGeom.SetStageMetersPerUnit(new_stage, 1.0)
    UsdGeom.SetStageUpAxis(new_stage, UsdGeom.Tokens.z)
    
    root_prim = UsdGeom.Xform.Define(new_stage, "/Robot")
    new_stage.SetDefaultPrim(root_prim.GetPrim())
    UsdPhysics.ArticulationRootAPI.Apply(root_prim.GetPrim())

    chassis_wpos = comp_data["Chassis"]["world_pos"]

    # 3. Create Links
    for key, data in comp_data.items():
        rel_pos = data["world_pos"] - chassis_wpos
        new_path = f"/Robot/{key}"
        new_link = UsdGeom.Xform.Define(new_stage, new_path)
        new_link.AddTranslateOp().Set(rel_pos)
        new_link.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(data["world_rot"])
        
        # Physics
        UsdPhysics.RigidBodyAPI.Apply(new_link.GetPrim())
        mass = UsdPhysics.MassAPI.Apply(new_link.GetPrim())
        mass.GetMassAttr().Set(4.0 if key == "Chassis" else 0.1)
        UsdPhysics.CollisionAPI.Apply(new_link.GetPrim())
        
        # Mesh
        new_mesh = UsdGeom.Mesh.Define(new_stage, f"{new_path}/Visual")
        old_mesh = data["mesh_prim"]
        points = old_mesh.GetAttribute("points").Get()
        
        pts = old_mesh.GetAttribute("points").Get()
        c = Gf.Vec3f(0)
        for p in pts: c += p
        c /= len(pts)
        
        new_pts = []
        for p in pts:
            # Scale to meters (original meshes were in cm)
            np = (p - c) * 0.01
            new_pts.append(np)
            
        new_mesh.GetPointsAttr().Set(new_pts)
        new_mesh.GetFaceVertexCountsAttr().Set(old_mesh.GetAttribute("faceVertexCounts").Get())
        new_mesh.GetFaceVertexIndicesAttr().Set(old_mesh.GetAttribute("faceVertexIndices").Get())

    # 4. Create Camera Prim directly on Chassis
    camera_prim = UsdGeom.Xform.Define(new_stage, "/Robot/Chassis/Camera")

    # 5. Create Joints
    def add_joint(name, p0, p1, axis, drive_type=None):
        j_path = f"/Robot/Joint_{name}"
        j_prim = new_stage.DefinePrim(j_path, "PhysicsRevoluteJoint")
        joint = UsdPhysics.RevoluteJoint(j_prim)
        joint.GetBody0Rel().SetTargets([f"/Robot/{p0}"])
        joint.GetBody1Rel().SetTargets([f"/Robot/{p1}"])
        
        w0 = comp_data[p0]["world_pos"]
        w1 = comp_data[p1]["world_pos"]
        rot0 = comp_data[p0]["world_rot"]
        
        lpos0 = rot0.GetInverse().Transform(w1 - w0)
        joint.GetLocalPos0Attr().Set(Gf.Vec3f(lpos0))
        joint.GetLocalPos1Attr().Set(Gf.Vec3f(0,0,0))
        joint.GetAxisAttr().Set(axis)
        
        if drive_type:
            drive = UsdPhysics.DriveAPI.Apply(j_prim, "angular")
            drive.GetTypeAttr().Set(drive_type)
            drive.GetStiffnessAttr().Set(1000.0 if drive_type == "position" else 0.0)
            drive.GetDampingAttr().Set(10.0)
            drive.GetMaxForceAttr().Set(1000.0)

    add_joint("Steer_L", "Chassis", "Knuckle_L", "Z", "position")
    add_joint("Steer_R", "Chassis", "Knuckle_R", "Z", "position")
    add_joint("Drive_FL", "Knuckle_L", "Wheel_FL", "Y", "velocity")
    add_joint("Drive_FR", "Knuckle_R", "Wheel_FR", "Y", "velocity")
    add_joint("Drive_RL", "Chassis", "Wheel_RL", "Y", "velocity")
    add_joint("Drive_RR", "Chassis", "Wheel_RR", "Y", "velocity")

    new_stage.GetRootLayer().Save()
    print(f"Flat Baked Robot saved to: {output_path}")

def main():
    import torch # Needed for heuristic
    bake_flat(args_cli.usd_path, args_cli.output_path)
    simulation_app.close()

if __name__ == "__main__":
    main()
