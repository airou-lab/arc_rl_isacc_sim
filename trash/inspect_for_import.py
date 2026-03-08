
import os
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

import omni.usd
from pxr import Usd, UsdGeom

def inspect_usd(path, label):
    print(f"\n--- Inspecting {label}: {path} ---")
    omni.usd.get_context().open_stage(path)
    simulation_app.update()
    stage = omni.usd.get_context().get_stage()
    
    for prim in stage.Traverse():
        # Look for Robot and ActionGraph
        if "f1tenth" in prim.GetName().lower() or "actiongraph" in prim.GetName().lower():
            print(f" Found: {prim.GetPath()} ({prim.GetTypeName()})")
        
        # Also print top level children
        if prim.GetPath().GetParentPath() == "/":
            print(f" Root Level Prim: {prim.GetPath()} ({prim.GetTypeName()})")

old_world = "/home/arika/Documents/arcpro/arcpro_system/src/examples/ARCPro_RL/arc_rl_isacc_sim/isacc_sim_usd/World0.usd"
new_world = "/home/arika/Documents/arcpro/arcpro_system/src/examples/ARCPro_RL/arc_rl_isacc_sim/openStreetUSD/arcpro_RL_open_street_sim.usd"

inspect_usd(old_world, "World0 (Source)")
inspect_usd(new_world, "Open Street (Target)")

simulation_app.close()
