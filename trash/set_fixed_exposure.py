
import os
from isaacsim import SimulationApp

# Initialize simulation app
simulation_app = SimulationApp({"headless": True})

import omni.usd
from pxr import Usd, UsdGeom, Gf, Sdf

# Load stage
usd_path = "/home/arika/Documents/arcpro/arcpro_system/src/examples/ARCPro_RL/arc_rl_isacc_sim/isacc_sim_usd/World0.usd"
omni.usd.get_context().open_stage(usd_path)
simulation_app.update()

stage = omni.usd.get_context().get_stage()

robot_path = "/World/F1Tenth"
cameras = ["Camera_Left", "Camera_Right", "Camera_Chase"]

print("\n--- Setting Fixed Low Exposure ---")
for cam_name in cameras:
    cam_path = f"{robot_path}/Rigid_Bodies/Chassis/{cam_name}"
    prim = stage.GetPrimAtPath(cam_path)
    if prim.IsValid():
        print(f"Configuring {cam_name}...")
        exposure_attr = prim.GetAttribute("exposure")
        if not exposure_attr.IsValid():
            exposure_attr = prim.CreateAttribute("exposure", Sdf.ValueTypeNames.Float)
        exposure_attr.Set(0.0) 
        
        fstop = prim.GetAttribute("fStop")
        if fstop.IsValid():
            fstop.Set(32.0) 

# Save stage
omni.usd.get_context().save_stage()
print("\nExposure fixed and stage saved.")

simulation_app.close()
