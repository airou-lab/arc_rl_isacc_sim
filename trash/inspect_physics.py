
import os
from isaacsim import SimulationApp

# Initialize simulation app
simulation_app = SimulationApp({"headless": True})

import omni.usd
from pxr import Usd, UsdPhysics, PhysxSchema

# Load stage
usd_path = "/home/arika/Documents/arcpro/arcpro_system/src/examples/ARCPro_RL/arc_rl_isacc_sim/isacc_sim_usd/World0.usd"
print(f"Loading stage: {usd_path}")
omni.usd.get_context().open_stage(usd_path)
simulation_app.update()

stage = omni.usd.get_context().get_stage()

joints = [
    "/World/F1Tenth/Joints/Wheel__Knuckle__Front_Left",
    "/World/F1Tenth/Joints/Wheel__Knuckle__Front_Right",
    "/World/F1Tenth/Joints/Wheel__Upright__Rear_Left",
    "/World/F1Tenth/Joints/Wheel__Upright__Rear_Right",
    "/World/F1Tenth/Joints/Knuckle__Upright__Front_Left"
]

for joint_path in joints:
    prim = stage.GetPrimAtPath(joint_path)
    if not prim.IsValid():
        print(f"Joint {joint_path} not found.")
        continue
    
    print(f"--- Physics for {joint_path} ---")
    drive = UsdPhysics.DriveAPI.Get(prim, "angular")
    if drive:
        print(f"  Drive Type: {drive.GetTypeAttr().Get()}")
        print(f"  Max Force: {drive.GetMaxForceAttr().Get()}")
        print(f"  Stiffness: {drive.GetStiffnessAttr().Get()}")
        print(f"  Damping: {drive.GetDampingAttr().Get()}")
    else:
        # Try generic "drive" or "linear" if angular fails
        print("  No 'angular' drive found. Checking all DriveAPIs...")
        for api in prim.GetAppliedSchemas():
            if "DriveAPI" in str(api):
                print(f"  Found API: {api}")

    # Check Physx properties
    physx_joint = PhysxSchema.PhysxJointAPI.Get(stage, joint_path)
    if physx_joint:
        print(f"  PhysX Max Force: {physx_joint.GetForceLimitAttr().Get()}")
        print(f"  PhysX Max Torque: {physx_joint.GetTorqueLimitAttr().Get()}")

simulation_app.close()
