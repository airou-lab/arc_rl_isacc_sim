
import os
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

import omni.usd
from pxr import Usd, UsdGeom

def check_ground():
    path = "/home/arika/Documents/arcpro/arcpro_system/src/examples/ARCPro_RL/arc_rl_isacc_sim/openStreetUSD/arcpro_RL_open_street_sim.usd"
    omni.usd.get_context().open_stage(path)
    simulation_app.update()
    stage = omni.usd.get_context().get_stage()
    
    print("\n--- Checking Root Level Meshes ---")
    for prim in stage.GetPrimAtPath("/World").GetChildren():
        if prim.IsA(UsdGeom.Mesh) or prim.IsA(UsdGeom.Xform):
            bbox = UsdGeom.Imageable(prim).ComputeWorldBound(0, "default")
            range = bbox.GetRange()
            print(f"Prim: {prim.GetPath()} | Z Range: {range.GetMin()[2]} to {range.GetMax()[2]}")

check_ground()
simulation_app.close()
