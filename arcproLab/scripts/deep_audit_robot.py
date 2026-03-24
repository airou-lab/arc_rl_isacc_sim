# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Deep Audit of Robot USD for Physics and Joints.")
parser.add_argument("usd_path", type=str, help="Path to the USD file.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

from pxr import Usd, UsdPhysics, UsdGeom, PhysxSchema, Gf

def audit_joints(stage):
    print("\n=== AUDIT 1: JOINT PROPERTIES ===")
    for prim in stage.Traverse():
        if prim.IsA(UsdPhysics.Joint):
            print(f"\nJoint: {prim.GetPath()}")
            # Check for drive
            drive_found = False
            for child in prim.GetChildren():
                if child.HasAPI(UsdPhysics.DriveAPI):
                    drive = UsdPhysics.DriveAPI(child)
                    print(f"  [DRIVE] {child.GetName()}: Type={drive.GetDriveTypeAttr().Get()}, Stiffness={drive.GetStiffnessAttr().Get()}, Damping={drive.GetDampingAttr().Get()}")
                    drive_found = True
            if not drive_found:
                print("  [WARNING] No Drive API found on this joint.")

            # Check for Physx Joint properties (Friction)
            if prim.HasAPI(PhysxSchema.PhysxJointAPI):
                physx_joint = PhysxSchema.PhysxJointAPI(prim)
                print(f"  [PHYSX] Joint Friction: {physx_joint.GetJointFrictionAttr().Get()}")
            
            # Check for Revolute limits
            if prim.IsA(UsdPhysics.RevoluteJoint):
                rev = UsdPhysics.RevoluteJoint(prim)
                print(f"  [LIMITS] {rev.GetLowerLimitAttr().Get()} to {rev.GetUpperLimitAttr().Get()}")

def audit_collisions(stage):
    print("\n=== AUDIT 2: COLLISION GEOMETRY ===")
    for prim in stage.Traverse():
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            geom = UsdGeom.Imageable(prim)
            purpose = geom.GetPurposeAttr().Get()
            is_visible = geom.GetVisibilityAttr().Get() == "inherited"
            
            print(f"Collision Prim: {prim.GetPath()}")
            print(f"  Purpose: {purpose}, Visible: {is_visible}")
            
            if prim.HasAPI(PhysxSchema.PhysxCollisionAPI):
                px_coll = PhysxSchema.PhysxCollisionAPI(prim)
                print(f"  Contact Offset: {px_coll.GetContactOffsetAttr().Get()}")
                print(f"  Rest Offset: {px_coll.GetRestOffsetAttr().Get()}")

def audit_mass(stage):
    print("\n=== AUDIT 3: MASS PROPERTIES ===")
    total_mass = 0
    for prim in stage.Traverse():
        if prim.HasAPI(UsdPhysics.MassAPI):
            mass_api = UsdPhysics.MassAPI(prim)
            m = mass_api.GetMassAttr().Get()
            if m:
                print(f"Prim: {prim.GetPath()} | Mass: {m}")
                total_mass += m
            
            com = mass_api.GetCenterOfMassAttr().Get()
            if com:
                print(f"  CoM: {com}")
            
            inertia = mass_api.GetDiagonalInertiaAttr().Get()
            if inertia:
                print(f"  Inertia: {inertia}")
    print(f"\nCalculated Total Mass: {total_mass}")

def main():
    print(f"Opening stage: {args_cli.usd_path}")
    stage = Usd.Stage.Open(args_cli.usd_path)
    
    audit_joints(stage)
    audit_collisions(stage)
    audit_mass(stage)

    simulation_app.close()

if __name__ == "__main__":
    main()
