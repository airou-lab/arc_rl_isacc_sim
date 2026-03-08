
import os
from isaacsim import SimulationApp

# Start simulation app
simulation_app = SimulationApp({"headless": True})

import omni.usd
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.stage import add_reference_to_stage
from pxr import Usd, UsdGeom

def add_street_to_world0():
    usd_path = "/home/arika/Documents/arcpro/arcpro_system/src/examples/ARCPro_RL/arc_rl_isacc_sim/isacc_sim_usd/World0.usd"
    print(f"Loading World0: {usd_path}")
    omni.usd.get_context().open_stage(usd_path)
    simulation_app.update()

    stage = omni.usd.get_context().get_stage()

    # 1. Add Simple Road as a Reference
    assets_root = get_assets_root_path()
    road_url = f"{assets_root}/Isaac/Environments/Simple_Road/simple_road.usd"
    print(f"Adding Street Reference: {road_url}")
    
    # Place it under /World/Street
    add_reference_to_stage(usd_path=road_url, prim_path="/World/Street")

    # 2. Ensure /World/Street is at ground level [0,0,0]
    street_prim = stage.GetPrimAtPath("/World/Street")
    if street_prim.IsValid():
        xform = UsdGeom.Xformable(street_prim)
        # Clear any existing transforms and set to identity
        xform.ClearXformOpOrder()
        xform.AddTranslateOp().Set((0, 0, 0))

    # 3. Save World0
    omni.usd.get_context().save_stage()
    print("World0.usd updated with Street geometry.")

    simulation_app.close()

if __name__ == "__main__":
    add_street_to_world0()
