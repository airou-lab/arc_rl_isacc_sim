
import os
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

import omni.usd
from pxr import Usd, Sdf

def check():
    path = "/home/arika/Documents/arcpro/arcpro_system/src/examples/ARCPro_RL/arc_rl_isacc_sim/openStreetUSD/arcpro_RL_open_street_sim.usd"
    omni.usd.get_context().open_stage(path)
    simulation_app.update()
    stage = omni.usd.get_context().get_stage()
    
    prim_path = "/World/F1Tenth/ActionGraph/setCamera"
    prim = stage.GetPrimAtPath(prim_path)
    if prim.IsValid():
        print(f"\n--- Checking {prim_path} ---")
        rel = prim.GetRelationship("inputs:cameraPrim")
        if rel.IsValid():
            print("SUCCESS: inputs:cameraPrim is a RELATIONSHIP.")
            print(f" Targets: {rel.GetTargets()}")
        else:
            print("inputs:cameraPrim is NOT a relationship.")
            attr = prim.GetAttribute("inputs:cameraPrim")
            if attr.IsValid():
                print(f"It is an ATTRIBUTE of type {attr.GetTypeName()}")
            else:
                print("It is NEITHER a relationship nor an attribute.")

check()
simulation_app.close()
