
import os
import numpy as np
from isaacsim import SimulationApp

# Initialize simulation app
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.articulations import Articulation
import omni.usd

def test_robot_response():
    print("\n[UAT] Starting Direct Physics Response Test (GUI ENABLED)...")
    
    # 1. Load stage
    usd_path = "/home/arika/Documents/arcpro/arcpro_system/src/examples/ARCPro_RL/arc_rl_isacc_sim/isacc_sim_usd/World0.usd"
    omni.usd.get_context().open_stage(usd_path)
    simulation_app.update()

    # 2. Initialize World
    world = World()
    world.reset()

    # 3. Setup Robot
    robot_path = "/World/F1Tenth"
    robot = Articulation(robot_path)
    world.scene.add(robot)
    world.reset()

    # 4. Command Movement (Throttle Indices: 25, 24, 16, 13)
    throttle_indices = [25, 24, 16, 13]
    target_velocities = np.zeros(robot.num_dof)
    for idx in throttle_indices:
        target_velocities[idx] = 100.0

    print(f"[UAT] Applying 100 rad/s to throttle joints for 1000 steps...")
    
    for i in range(1000):
        robot.set_joint_velocities(target_velocities)
        world.step(render=True) # Render must be True for GUI
        
        if i % 100 == 0:
            vel = robot.get_linear_velocity()
            speed = np.linalg.norm(vel)
            print(f"[UAT] Step {i}: Speed = {speed:.2f} m/s")

    final_speed = np.linalg.norm(robot.get_linear_velocity())
    print(f"\n[UAT] Final Speed: {final_speed:.2f} m/s")
    
    if final_speed > 3.0:
        print("RESULT: SUCCESS. Physics engine is responsive.")
    else:
        print("RESULT: FAILURE. Physics engine is not responding as expected.")

    simulation_app.close()

if __name__ == "__main__":
    test_robot_response()
