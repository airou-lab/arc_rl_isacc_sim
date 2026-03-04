
import os
from isaacsim import SimulationApp

# Initialize simulation app
simulation_app = SimulationApp({"headless": True})

from omni.isaac.core import World
from omni.isaac.core.articulations import Articulation
import omni.usd

# Load stage
usd_path = "/home/arika/Documents/arcpro/arcpro_system/src/examples/ARCPro_RL/arc_rl_isacc_sim/isacc_sim_usd/World0.usd"
print(f"Loading stage: {usd_path}")
omni.usd.get_context().open_stage(usd_path)
simulation_app.update()

# Initialize World (adds Physics Scene if missing)
world = World()
world.reset()

# Initialize Articulation
robot_path = "/World/F1Tenth"
robot = Articulation(robot_path)
world.scene.add(robot)
world.reset() # This initializes articulations in the scene

print(f"--- Joint Indices for {robot_path} ---")
# Accessing dof_names and indices
if hasattr(robot, "dof_names"):
    for name in robot.dof_names:
        print(f"Joint: {name} | Index: {robot.get_dof_index(name)}")
else:
    print("Could not retrieve DOF names. Checking handle...")
    print(robot.dof_names_to_indices)

simulation_app.close()
