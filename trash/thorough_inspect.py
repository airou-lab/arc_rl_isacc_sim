
import os
from isaacsim import SimulationApp

# Initialize simulation app
simulation_app = SimulationApp({"headless": True})

import omni.usd
from pxr import Usd, UsdGeom

# Load stage
usd_path = "/home/arika/Documents/arcpro/arcpro_system/src/examples/ARCPro_RL/arc_rl_isacc_sim/isacc_sim_usd/World0.usd"
print(f"Loading stage: {usd_path}")
omni.usd.get_context().open_stage(usd_path)
simulation_app.update()

stage = omni.usd.get_context().get_stage()

print("\n--- Searching for ALL Cameras ---")
for prim in stage.Traverse():
    if prim.IsA(UsdGeom.Camera):
        print(f"Found Camera: {prim.GetPath()}")

print("\n--- Searching for F1Tenth Structure ---")
robot_prim = stage.GetPrimAtPath("/World/F1Tenth")
if robot_prim.IsValid():
    for prim in Usd.PrimRange(robot_prim):
        if "camera" in prim.GetName().lower():
            print(f"Robot Camera Match: {prim.GetPath()} ({prim.GetTypeName()})")
else:
    print("F1Tenth prim not found at /World/F1Tenth")

simulation_app.close()
