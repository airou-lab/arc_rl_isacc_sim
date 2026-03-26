# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Reconstruct Robot Articulation properly.")
parser.add_argument("usd_path", type=str, help="Path to the source USD file.")
parser.add_argument("output_path", type=str, help="Path to the output USD file.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

from pxr import Usd, UsdGeom, Gf, UsdPhysics

def reconstruct_robot(usd_path, output_path):
    print(f"Opening stage: {usd_path}")
    stage = Usd.Stage.Open(usd_path)
    
    # We define the correct metric positions (in units where 100 units = 1m? No, let's use MetersPerUnit = 1.0)
    # F1Tenth dimensions in meters:
    # Wheelbase: 0.325m (+0.165, -0.160)
    # Track: 0.200m (+0.100, -0.100)
    # Wheel Radius: 0.033m
    
    wb_f = 0.165
    wb_r = -0.160
    tw_l = 0.100
    tw_r = -0.100
    h = 0.033
    
    # Map from prim name to target world position
    pos_map = {
        "Chassis": Gf.Vec3d(0, 0, h),
        "Wheel_Front_Left": Gf.Vec3d(wb_f, tw_l, h),
        "Wheel_Front_Right": Gf.Vec3d(wb_f, tw_r, h),
        "Wheel_Rear_Left": Gf.Vec3d(wb_r, tw_l, h),
        "Wheel_Rear_Right": Gf.Vec3d(wb_r, tw_r, h),
        "Steering_Knuckle_Left": Gf.Vec3d(wb_f, tw_l, h),
        "Steering_Knuckle_Right": Gf.Vec3d(wb_f, tw_r, h),
        # Add others if needed
    }

    # Set Stage to Meters
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)

    # 1. Update Rigidbody Positions and Reset Meshes
    for prim in stage.Traverse():
        name = prim.GetName()
        if name in pos_map:
            print(f"Positioning {prim.GetPath()} to {pos_map[name]}")
            xform = UsdGeom.Xformable(prim)
            xform.ClearXformOpOrder()
            xform.AddTranslateOp().Set(pos_map[name])
            
            # If it's a mesh, we might need to rescale it to match the new 1.0 MPU
            # Original mesh was 10.4 units wide. In 0.01 MPU that was 0.104m.
            # Now in 1.0 MPU, it's 10.4m! 100x too big.
            # So we MUST scale meshes down by 0.01.
            if prim.IsA(UsdGeom.Mesh):
                xform.AddScaleOp().Set(Gf.Vec3f(0.01, 0.01, 0.01))

    # 2. Fix Joints
    # Joints should now have localPos = 0 if they connect the centers of these bodies
    for prim in stage.Traverse():
        if prim.IsA(UsdPhysics.Joint):
            joint = UsdPhysics.Joint(prim)
            print(f"Zeroing joint frames for {prim.GetPath()}")
            joint.GetLocalPos0Attr().Set(Gf.Vec3f(0,0,0))
            joint.GetLocalPos1Attr().Set(Gf.Vec3f(0,0,0))
            joint.GetLocalRot0Attr().Set(Gf.Quatf(1,0,0,0))
            joint.GetLocalRot1Attr().Set(Gf.Quatf(1,0,0,0))
            
            # Re-apply Axis and Drive
            if prim.IsA(UsdPhysics.RevoluteJoint):
                rev = UsdPhysics.RevoluteJoint(prim)
                if "Wheel" in prim.GetName():
                    rev.GetAxisAttr().Set("Y")
                    drive = UsdPhysics.DriveAPI.Apply(prim, "angular")
                    drive.GetTypeAttr().Set("velocity")
                else:
                    rev.GetAxisAttr().Set("Z")
                    drive = UsdPhysics.DriveAPI.Apply(prim, "angular")
                    drive.GetTypeAttr().Set("position")

    stage.GetRootLayer().Export(output_path)
    print(f"Reconstructed Robot saved to: {output_path}")

def main():
    reconstruct_robot(args_cli.usd_path, args_cli.output_path)
    simulation_app.close()

if __name__ == "__main__":
    main()
