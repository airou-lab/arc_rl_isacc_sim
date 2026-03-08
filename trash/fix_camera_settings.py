
import os
from isaacsim import SimulationApp

# Initialize simulation app
simulation_app = SimulationApp({"headless": True})

import omni.usd
from pxr import Usd, UsdGeom

# Load stage
usd_path = "/home/arika/Documents/arcpro/arcpro_system/src/examples/ARCPro_RL/arc_rl_isacc_sim/isacc_sim_usd/World0.usd"
omni.usd.get_context().open_stage(usd_path)
simulation_app.update()

stage = omni.usd.get_context().get_stage()

robot_path = "/World/F1Tenth"
cameras = ["Camera_Left", "Camera_Right", "Camera_Chase"]

print("\n--- Fixing Camera Optics ---")
for cam_name in cameras:
    cam_path = f"{robot_path}/Rigid_Bodies/Chassis/{cam_name}"
    prim = stage.GetPrimAtPath(cam_path)
    if prim.IsValid():
        print(f"Fixing {cam_name}...")
        # Increase fStop to let in LESS light (dim the image)
        fstop_attr = prim.GetAttribute("fStop")
        if fstop_attr.IsValid():
            fstop_attr.Set(16.0) # High f-stop = darker image
        
        # Lower focal length slightly to see more of the track
        focal_attr = prim.GetAttribute("focalLength")
        if focal_attr.IsValid():
            focal_attr.Set(18.0) # Wider angle

        # Explicitly set shutter to be faster
        shutter_attr = prim.GetAttribute("shutter:open")
        if shutter_attr.IsValid():
            shutter_attr.Set(0.0)
        shutter_close = prim.GetAttribute("shutter:close")
        if shutter_close.IsValid():
            shutter_close.Set(0.0)

# Save stage
omni.usd.get_context().save_stage()
print("\nCamera settings fixed and stage saved.")

simulation_app.close()
