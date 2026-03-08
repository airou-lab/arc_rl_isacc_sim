
import os
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

import omni.usd
from pxr import Usd, UsdGeom

def inspect_robot(path):
    print(f"\n--- Inspecting Robot at {path} ---")
    omni.usd.get_context().open_stage(path)
    simulation_app.update()
    stage = omni.usd.get_context().get_stage()
    
    robot_prim = stage.GetPrimAtPath("/World/F1Tenth")
    if not robot_prim.IsValid():
        print("Robot prim not found!")
        return

    for prim in Usd.PrimRange(robot_prim):
        if "camera" in prim.GetName().lower():
            print(f" Camera Found: {prim.GetPath()} ({prim.GetTypeName()})")

target_world = "/home/arika/Documents/arcpro/arcpro_system/src/examples/ARCPro_RL/arc_rl_isacc_sim/openStreetUSD/arcpro_RL_open_street_sim.usd"
inspect_robot(target_world)

simulation_app.close()
