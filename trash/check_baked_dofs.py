from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

from omni.isaac.core import World
from omni.isaac.core.articulations import ArticulationView
import numpy as np

def check_baked_dofs():
    print("[Audit] Loading Baked Model for DOF check...")
    world = World(stage_units_in_meters=1.0)
    usd_path = "/home/arika/Documents/arcpro/arcpro_system/src/examples/ARCPro_RL/arc_rl_isacc_sim/f1tenth_trainer/assets/F1Tenth_Metric_Baked.usd"
    
    from omni.isaac.core.utils.stage import add_reference_to_stage
    add_reference_to_stage(usd_path=usd_path, prim_path="/Full_Car")
    
    robots = ArticulationView("/Full_Car")
    world.scene.add(robots)
    world.reset()
    robots.initialize()
    
    print("\n--- DOF NAMES & INDICES ---")
    for i, name in enumerate(robots.dof_names):
        print(f"{i:2d}: {name}")
        
    simulation_app.close()

if __name__ == "__main__":
    check_baked_dofs()
