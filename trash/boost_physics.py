
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
    "/World/F1Tenth/Joints/Knuckle__Upright__Front_Left",
    "/World/F1Tenth/Joints/Knuckle__Upright__Front_Right"
]

print("--- Boosting Joint Forces ---")
for joint_path in joints:
    prim = stage.GetPrimAtPath(joint_path)
    if not prim.IsValid():
        print(f"Joint {joint_path} not found.")
        continue
    
    # 1. Update UsdPhysics Drive
    drive = UsdPhysics.DriveAPI.Get(prim, "angular")
    if drive:
        print(f"  Updating {joint_path} Angular Drive...")
        drive.GetMaxForceAttr().Set(1000.0) # Boost to 1000
    
    # 2. Update Physx Joint API (Redundant but safe)
    physx_joint = PhysxSchema.PhysxJointAPI.Get(stage, joint_path)
    if physx_joint:
        print(f"  Updating {joint_path} PhysX Limits...")
        physx_joint.GetForceLimitAttr().Set(1000.0)
        physx_joint.GetTorqueLimitAttr().Set(1000.0)

# Save the stage
omni.usd.get_context().save_stage()
print("Stage saved with boosted physics.")

simulation_app.close()
