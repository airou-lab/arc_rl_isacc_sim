
import os
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

import omni.usd
from pxr import Usd, UsdGeom

def find_line():
    path = "/home/arika/Documents/arcpro/arcpro_system/src/examples/ARCPro_RL/arc_rl_isacc_sim/openStreetUSD/arcpro_RL_open_street_sim.usd"
    omni.usd.get_context().open_stage(path)
    simulation_app.update()
    stage = omni.usd.get_context().get_stage()
    
    print("\n--- Searching for Yellow Line/Road Center ---")
    for prim in stage.Traverse():
        name = prim.GetName().lower()
        if "line" in name or "marking" in name or "center" in name:
            bbox = UsdGeom.Imageable(prim).ComputeWorldBound(0, "default")
            pos = bbox.GetRange().GetMidpoint()
            print(f"Marking: {prim.GetPath()} | Center: {pos}")

find_line()
simulation_app.close()
