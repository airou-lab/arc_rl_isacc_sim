# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Final Robot Reconstruction with Spec Dimensions.")
parser.add_argument("usd_path", type=str, help="Path to the source USD file.")
parser.add_argument("output_path", type=str, help="Path to the output USD file.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

from pxr import Usd, UsdGeom, Gf, UsdPhysics

def spec_reconstruct(usd_path, output_path):
    print(f"Opening stage: {usd_path}")
    stage = Usd.Stage.Open(usd_path)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)

    # F1Tenth spec dimensions (meters)
    # Origin at center of chassis
    wb = 0.325
    tw = 0.200
    wr = 0.052 # radius
    
    # Body positions
    body_specs = {
        "Chassis": Gf.Vec3d(0, 0, 0.05), # chassis center at 5cm
        "Wheel_Front_Left": Gf.Vec3d(wb/2, tw/2, wr),
        "Wheel_Front_Right": Gf.Vec3d(wb/2, -tw/2, wr),
        "Wheel_Rear_Left": Gf.Vec3d(-wb/2, tw/2, wr),
        "Wheel_Rear_Right": Gf.Vec3d(-wb/2, -tw/2, wr),
        "Steering_Knuckle_Left": Gf.Vec3d(wb/2, tw/2, wr),
        "Steering_Knuckle_Right": Gf.Vec3d(wb/2, -tw/2, wr),
    }

    # 1. Center all meshes and move parents to spec pos
    for prim in stage.Traverse():
        if prim.IsA(UsdGeom.Mesh):
            mesh = UsdGeom.Mesh(prim)
            points = mesh.GetPointsAttr().Get()
            if not points: continue
            
            center = Gf.Vec3f(0)
            for p in points: center += p
            center /= len(points)
            
            # Center vertices
            mesh.GetPointsAttr().Set([p - center for p in points])
            
            # Find closest matching component
            parent = prim.GetParent()
            pname = parent.GetName()
            match = None
            for key in body_specs:
                if key in pname:
                    match = key
                    break
            
            if match:
                pxform = UsdGeom.Xformable(parent)
                pxform.ClearXformOpOrder()
                pxform.AddTranslateOp().Set(body_specs[match])
                print(f"Positioned {parent.GetPath()} to {body_specs[match]} based on spec {match}")
                
                # Scale mesh to 0.01 (original unit correction)
                cxform = UsdGeom.Xformable(prim)
                cxform.ClearXformOpOrder()
                cxform.AddScaleOp().Set(Gf.Vec3f(0.01, 0.01, 0.01))
                
                # Ensure RB and Mass
                UsdPhysics.RigidBodyAPI.Apply(parent)
                mass = UsdPhysics.MassAPI.Apply(parent)
                mass.GetMassAttr().Set(4.0 if "Chassis" in pname else 0.1)
                UsdPhysics.CollisionAPI.Apply(parent)

    # 2. Fix Joints
    for prim in stage.Traverse():
        if prim.IsA(UsdPhysics.Joint):
            joint = UsdPhysics.Joint(prim)
            t0 = joint.GetBody0Rel().GetTargets()
            t1 = joint.GetBody1Rel().GetTargets()
            if t0 and t1:
                # Find spec positions for these bodies
                p0, p1 = None, None
                for key, pos in body_specs.items():
                    if key in t0[0].pathString: p0 = pos
                    if key in t1[0].pathString: p1 = pos
                
                if p0 is not None and p1 is not None:
                    local_pos0 = p1 - p0
                    joint.GetLocalPos0Attr().Set(Gf.Vec3f(local_pos0))
                    joint.GetLocalPos1Attr().Set(Gf.Vec3f(0,0,0))
                    print(f"Fixed Joint {prim.GetPath()} relative offset to {local_pos0}")

            if prim.IsA(UsdPhysics.RevoluteJoint):
                rev = UsdPhysics.RevoluteJoint(prim)
                rev.GetAxisAttr().Set("Y" if "Wheel" in prim.GetName() else "Z")

    stage.GetRootLayer().Export(output_path)
    print(f"Spec Reconstructed Robot saved to: {output_path}")

def main():
    spec_reconstruct(args_cli.usd_path, args_cli.output_path)
    simulation_app.close()

if __name__ == "__main__":
    main()
