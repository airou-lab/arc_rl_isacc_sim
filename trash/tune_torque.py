
import os
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

import omni.usd
from pxr import Usd, UsdPhysics

def fix_torque():
    path = "/home/arika/Documents/arcpro/arcpro_system/src/examples/ARCPro_RL/arc_rl_isacc_sim/openStreetUSD/arcpro_RL_open_street_sim.usd"
    omni.usd.get_context().open_stage(path)
    simulation_app.update()
    stage = omni.usd.get_context().get_stage()
    
    print("\n--- Tuning Torque for Stability ---")
    for prim in stage.Traverse():
        if prim.IsA(UsdPhysics.DriveAPI):
            # Set a more reasonable max force
            drive = UsdPhysics.DriveAPI(prim)
            drive.CreateMaxForceAttr(100.0)
            print(f"Set maxForce=100.0 on {prim.GetPath()}")

    omni.usd.get_context().save_stage()
    print("Torque updated.")

fix_torque()
simulation_app.close()
