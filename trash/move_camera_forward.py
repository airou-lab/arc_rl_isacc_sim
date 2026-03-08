
import os
from isaacsim import SimulationApp

# Initialize simulation app
simulation_app = SimulationApp({"headless": True})

import omni.usd
from pxr import Usd, UsdGeom, Gf

# Load stage
usd_path = "/home/arika/Documents/arcpro/arcpro_system/src/examples/ARCPro_RL/arc_rl_isacc_sim/isacc_sim_usd/World0.usd"
omni.usd.get_context().open_stage(usd_path)
simulation_app.update()

stage = omni.usd.get_context().get_stage()

robot_path = "/World/F1Tenth"
cameras = ["Camera_Left", "Camera_Right"]

print("\n--- Moving Cameras FORWARD (Corrected) ---")
for cam_name in cameras:
    cam_path = f"{robot_path}/Rigid_Bodies/Chassis/{cam_name}"
    prim = stage.GetPrimAtPath(cam_path)
    if prim.IsValid():
        print(f"Moving {cam_name}...")
        xform = UsdGeom.Xformable(prim)
        
        # Clear existing xform ops to avoid stacking
        xform.ClearXformOpOrder()
        
        # Add a new translation op
        translate_op = xform.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble)
        # Move 0.25m forward (X), 0.0m right, 0.15m up (Z)
        translate_op.Set(Gf.Vec3d(0.25, 0.0, 0.15))
        
        # Also ensure it's pointing forward (X)
        rotate_op = xform.AddRotateXYZOp(UsdGeom.XformOp.PrecisionFloat)
        rotate_op.Set(Gf.Vec3f(0.0, 0.0, 0.0))

# Save stage
omni.usd.get_context().save_stage()
print("\nCameras moved and stage saved.")

simulation_app.close()
