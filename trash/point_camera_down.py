
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

print("\n--- Pointing Cameras at the Track ---")
for cam_name in cameras:
    cam_path = f"{robot_path}/Rigid_Bodies/Chassis/{cam_name}"
    prim = stage.GetPrimAtPath(cam_path)
    if prim.IsValid():
        print(f"Adjusting {cam_name} orientation...")
        xform = UsdGeom.Xformable(prim)
        
        # Point 15 degrees down
        # Standard Isaac camera: +X is forward, +Y is up, +Z is right (or similar)
        # We need to rotate around the horizontal axis.
        # Let's try setting a clean rotation.
        # Common convention for onboard: [0, 15, 0] or [0, 0, -15]
        
        # Clear existing rotations and set new one
        xform.SetRotate(Gf.Vec3f(0.0, 15.0, 0.0), UsdGeom.XformOp.PrecisionFloat)

# Save stage
omni.usd.get_context().save_stage()
print("\nCamera orientation updated and stage saved.")

simulation_app.close()
