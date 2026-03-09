
import os
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

import omni.usd
from pxr import Usd

def remove_ground():
    path = "/home/arika/Documents/arcpro/arcpro_system/src/examples/ARCPro_RL/arc_rl_isacc_sim/openStreetUSD/arcpro_RL_open_street_sim.usd"
    omni.usd.get_context().open_stage(path)
    simulation_app.update()
    stage = omni.usd.get_context().get_stage()
    
    ground_path = "/World/SolidGroundUSD"
    if stage.GetPrimAtPath(ground_path).IsValid():
        print(f"Removing {ground_path}...")
        stage.RemovePrim(ground_path)
        omni.usd.get_context().save_stage()
        print("World restored to original road geometry.")
    else:
        print("Solid USD plane not found.")

remove_ground()
simulation_app.close()
