
import os
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

import omni.usd
from omni.isaac.core.articulations import Articulation
from pxr import Usd, UsdGeom

def check_joints():
    path = "/home/arika/Documents/arcpro/arcpro_system/src/examples/ARCPro_RL/arc_rl_isacc_sim/openStreetUSD/arcpro_RL_open_street_sim.usd"
    omni.usd.get_context().open_stage(path)
    simulation_app.update()
    
    from omni.isaac.core import World
    world = World()
    
    robot_path = "/World/F1Tenth"
    art = Articulation(robot_path)
    world.scene.add(art)
    world.reset() # This initializes the articulation
    
    print(f"\n--- Articulation: {robot_path} ---")
    print(f"DOF Names: {art.dof_names}")
    print(f"Joint Names: {art.joint_names}")
    
    # Get current (default) positions
    pos = art.get_joint_positions()
    print(f"Default Positions: {pos}")

check_joints()
simulation_app.close()
