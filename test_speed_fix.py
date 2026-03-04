
import os
import time
import numpy as np
from isaacsim import SimulationApp

# Initialize simulation app (Headless for verification)
simulation_app = SimulationApp({"headless": True})

from omni.isaac.core import World
from omni.isaac.core.articulations import Articulation
import omni.usd

# Load stage
usd_path = "/home/arika/Documents/arcpro/arcpro_system/src/examples/ARCPro_RL/arc_rl_isacc_sim/isacc_sim_usd/World0.usd"
print(f"Loading stage: {usd_path}")
omni.usd.get_context().open_stage(usd_path)
simulation_app.update()

# Initialize World
world = World()
world.reset()

# Initialize Articulation
robot_path = "/World/F1Tenth"
robot = Articulation(robot_path)
world.scene.add(robot)
world.reset()

# Throttle indices confirmed earlier: [25, 24, 16, 13]
throttle_indices = [25, 24, 16, 13]

print("\n--- Starting Speed Test (50 steps of Full Throttle) ---")
# Apply full throttle (100 rad/s)
target_velocities = np.zeros(robot.num_dof)
for idx in throttle_indices:
    target_velocities[idx] = 100.0 # 100 rad/s

for i in range(50):
    robot.set_joint_velocities(target_velocities)
    world.step(render=False)
    
    # Get linear velocity of chassis
    # Articulation.get_linear_velocity() returns [x, y, z]
    vel = robot.get_linear_velocity()
    speed = np.linalg.norm(vel)
    
    if i % 10 == 0:
        print(f"Step {i}: Speed = {speed:.2f} m/s")

# Final check
vel = robot.get_linear_velocity()
final_speed = np.linalg.norm(vel)
print(f"Final Speed: {final_speed:.2f} m/s")

if final_speed > 1.0:
    print("\nSUCCESS: Robot reached responsive speed!")
else:
    print("\nFAILURE: Robot still moving slowly.")

simulation_app.close()
