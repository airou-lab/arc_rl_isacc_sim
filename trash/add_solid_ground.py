
import os
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

import omni.usd
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.stage import add_reference_to_stage
from pxr import Usd, UsdGeom, Gf

def add_ground():
    path = "/home/arika/Documents/arcpro/arcpro_system/src/examples/ARCPro_RL/arc_rl_isacc_sim/openStreetUSD/arcpro_RL_open_street_sim.usd"
    omni.usd.get_context().open_stage(path)
    simulation_app.update()
    stage = omni.usd.get_context().get_stage()
    
    # 1. Add a standard Physics Ground Plane at the road height
    # This is a specialized prim that is 100% solid
    ground_path = "/World/SolidGroundPlane"
    if not stage.GetPrimAtPath(ground_path).IsValid():
        print(f"Adding Solid Ground Plane at z=15.14...")
        from omni.isaac.core.objects import GroundPlane
        import numpy as np
        plane = GroundPlane(prim_path=ground_path, position=np.array([0, 0, 15.14]))
    else:
        print("Solid ground plane already exists.")

    omni.usd.get_context().save_stage()
    print("World updated with SolidGroundPlane.")

add_ground()
simulation_app.close()
