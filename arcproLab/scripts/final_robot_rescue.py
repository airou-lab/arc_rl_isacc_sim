# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Clean and Bake Robot Articulation (Surgical v2).")
parser.add_argument("usd_path", type=str, help="Path to the source USD file.")
parser.add_argument("output_path", type=str, help="Path to the output USD file.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

from pxr import Usd, UsdGeom, Gf, UsdPhysics, PhysxSchema

def rescue_robot_surgical(usd_path, output_path):
    print(f"Opening stage: {usd_path}")
    stage = Usd.Stage.Open(usd_path)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    
    # 1. Force Articulation Root
    root_prim = stage.GetPrimAtPath("/Full_Car")
    if root_prim:
        print(f"Applying ArticulationRootAPI to {root_prim.GetPath()}")
        UsdPhysics.ArticulationRootAPI.Apply(root_prim)
        PhysxSchema.PhysxArticulationAPI.Apply(root_prim)

    # 2. Define the ONLY rigid bodies allowed in the articulation (Xforms only)
    allowed_rb_names = [
        "Chassis",
        "Wheel_Front_Left", "Wheel_Front_Right",
        "Wheel_Rear_Left", "Wheel_Rear_Right",
        "Steering_Knuckle_Left", "Steering_Knuckle_Right"
    ]
    
    target_pos = {}
    
    # 3. First Pass: Mesh cleanup and scaling
    for prim in stage.Traverse():
        if prim.IsA(UsdGeom.Mesh):
            # ALWAYS remove RigidBodyAPI from Mesh prims to avoid nesting
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                print(f"Removing nested RigidBodyAPI from Mesh: {prim.GetPath()}")
                prim.RemoveAPI(UsdPhysics.RigidBodyAPI)
                if prim.HasAPI(UsdPhysics.MassAPI): prim.RemoveAPI(UsdPhysics.MassAPI)
            
            mesh = UsdGeom.Mesh(prim)
            points = mesh.GetPointsAttr().Get()
            if not points: continue
            
            center = Gf.Vec3f(0)
            for p in points: center += p
            center /= len(points)
            mesh.GetPointsAttr().Set([p - center for p in points])
            
            parent = prim.GetParent()
            pxform = UsdGeom.Xformable(parent)
            pxform.ClearXformOpOrder()
            metric_pos = Gf.Vec3d(center[0]*0.01, center[1]*0.01, center[2]*0.01)
            pxform.AddTranslateOp().Set(metric_pos)
            target_pos[parent.GetPath().pathString] = metric_pos
            
            # Rescale mesh to 0.01
            UsdGeom.Xformable(prim).ClearXformOpOrder()
            UsdGeom.Xformable(prim).AddScaleOp().Set(Gf.Vec3f(0.01, 0.01, 0.01))

    # 4. Second Pass: Surgical RigidBody control on Xforms
    for prim in stage.Traverse():
        if not prim.IsA(UsdGeom.Xform): continue
        
        name = prim.GetName()
        has_rb = prim.HasAPI(UsdPhysics.RigidBodyAPI)
        is_allowed = any(n == name for n in allowed_rb_names)
        
        if has_rb and not is_allowed:
            print(f"Removing prohibited RigidBodyAPI from Xform: {prim.GetPath()}")
            prim.RemoveAPI(UsdPhysics.RigidBodyAPI)
            if prim.HasAPI(UsdPhysics.MassAPI): prim.RemoveAPI(UsdPhysics.MassAPI)
        
        if is_allowed:
            print(f"Ensuring robust RigidBodyAPI for Xform: {prim.GetPath()}")
            UsdPhysics.RigidBodyAPI.Apply(prim)
            mass_api = UsdPhysics.MassAPI.Apply(prim)
            mass_api.GetMassAttr().Set(4.0 if name == "Chassis" else 0.1)
            # Add collision to the RB Xform (inherited by mesh)
            UsdPhysics.CollisionAPI.Apply(prim)

    # 5. Third Pass: Joint Reconstruction
    for prim in stage.Traverse():
        if prim.IsA(UsdPhysics.Joint):
            joint = UsdPhysics.Joint(prim)
            t0 = joint.GetBody0Rel().GetTargets()
            t1 = joint.GetBody1Rel().GetTargets()
            
            if t0 and t1:
                p0_path = t0[0].pathString
                p1_path = t1[0].pathString
                if p0_path in target_pos and p1_path in target_pos:
                    w0 = target_pos[p0_path]
                    w1 = target_pos[p1_path]
                    joint.GetLocalPos0Attr().Set(Gf.Vec3f(w1 - w0))
                    joint.GetLocalPos1Attr().Set(Gf.Vec3f(0,0,0))
                    
            if prim.IsA(UsdPhysics.RevoluteJoint):
                rev = UsdPhysics.RevoluteJoint(prim)
                rev.GetAxisAttr().Set("Y" if "Wheel" in prim.GetName() else "Z")

    stage.GetRootLayer().Export(output_path)
    print(f"Surgically Rescued Robot v2 saved to: {output_path}")

def main():
    rescue_robot_surgical(args_cli.usd_path, args_cli.output_path)
    simulation_app.close()

if __name__ == "__main__":
    main()
