
import os
import time
import numpy as np
from isaacsim import SimulationApp

# 1. Start Simulation App with GUI
simulation_app = SimulationApp({"headless": False})

import omni.usd
from omni.isaac.core import World
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.stage import add_reference_to_stage
from pxr import Usd, UsdGeom, UsdLux, Gf, Sdf

def scaffold_fixed_street():
    # 2. OPEN the road asset as the primary stage
    assets_root = get_assets_root_path()
    road_url = f"{assets_root}/Isaac/Environments/Simple_Road/simple_road.usd"
    print(f"[Step 1] Opening official road: {road_url}")
    omni.usd.get_context().open_stage(road_url)
    
    # Wait for resolve
    for _ in range(100): simulation_app.update()
    
    stage = omni.usd.get_context().get_stage()

    # 3. Add Robot Reference
    robot_source = "/home/arika/Documents/arcpro/arcpro_system/src/examples/ARCPro_RL/arc_rl_isacc_sim/f1tenth_trainer/assets/F1Tenth.usd"
    print(f"[Step 2] Referencing Robot: {robot_source}")
    # Root prim in simple_road is often /World or /Simple_Road
    root_path = "/World" if stage.GetPrimAtPath("/World").IsValid() else "/Simple_Road"
    add_reference_to_stage(usd_path=robot_source, prim_path=f"{root_path}/F1Tenth")

    # 4. Save locally
    save_path = "/home/arika/Documents/arcpro/arcpro_system/src/examples/ARCPro_RL/arc_rl_isacc_sim/isacc_sim_usd/World_Street_Fixed.usd"
    omni.usd.get_context().save_as_stage(save_path)
    print(f"\n[DONE] New world saved to {save_path}")

    # 5. KEEP OPEN
    print("\n--- SIMULATION OPEN FOR VERIFICATION ---")
    start_time = time.time()
    while time.time() - start_time < 60:
        simulation_app.update()

    simulation_app.close()

if __name__ == "__main__":
    scaffold_fixed_street()
