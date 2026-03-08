
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
cameras = ["Camera_Left", "Camera_Right", "Camera_Chase"]

for cam_name in cameras:
    cam_path = f"{robot_path}/Rigid_Bodies/Chassis/{cam_name}"
    prim = stage.GetPrimAtPath(cam_path)
    if prim.IsValid():
        print(f"\n--- {cam_name} Details ---")
        xform = UsdGeom.Xformable(prim)
        world_transform = xform.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        translation = world_transform.ExtractTranslation()
        print(f"  World Position: {translation}")
        
        # Check parent pose
        chassis_prim = stage.GetPrimAtPath(f"{robot_path}/Rigid_Bodies/Chassis")
        chassis_xform = UsdGeom.Xformable(chassis_prim)
        chassis_pos = chassis_xform.ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation()
        print(f"  Chassis World Position: {chassis_pos}")
        
        local_pos = translation - chassis_pos
        print(f"  Relative Position to Chassis: {local_pos}")

simulation_app.close()
