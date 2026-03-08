
import os
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

import omni.usd
import omni.graph.core as og

def verify():
    path = "/home/arika/Documents/arcpro/arcpro_system/src/examples/ARCPro_RL/arc_rl_isacc_sim/openStreetUSD/arcpro_RL_open_street_sim.usd"
    omni.usd.get_context().open_stage(path)
    simulation_app.update()
    
    attr_path = "/World/F1Tenth/ActionGraph/setCamera.inputs:cameraPrim"
    attr = og.Controller.get_attribute(attr_path)
    if attr:
        val = og.Controller.get(attr)
        print(f"\n--- Final Verification ---")
        print(f"Attribute: {attr_path}")
        print(f"Value: {val}")
        if val and len(val) > 0:
            print("SUCCESS: cameraPrim is correctly set.")
        else:
            print("FAILED: cameraPrim is EMPTY.")
    else:
        print("Attribute not found.")

verify()
simulation_app.close()
