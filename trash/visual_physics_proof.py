
import os
import time
import numpy as np
from isaacsim import SimulationApp

# Initialize simulation app with GUI
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.articulations import Articulation
import omni.usd

def run_proof():
    print("\n[UAT] Starting Visual Physics Proof...")
    
    # 1. Load stage
    usd_path = "/home/arika/Documents/arcpro/arcpro_system/src/examples/ARCPro_RL/arc_rl_isacc_sim/isacc_sim_usd/World0.usd"
    omni.usd.get_context().open_stage(usd_path)
    
    # Wait for stage to load fully
    for _ in range(100):
        simulation_app.update()

    # 2. Initialize World
    world = World()
    world.reset()

    # 3. Setup Robot
    robot_path = "/World/F1Tenth"
    robot = Articulation(robot_path)
    world.scene.add(robot)
    world.reset()

    # 4. Command Movement
    throttle_indices = [25, 24, 16, 13]
    target_velocities = np.zeros(robot.num_dof)
    for idx in throttle_indices:
        target_velocities[idx] = 100.0

    print(f"\n[UAT] WINDOW SHOULD BE OPEN. Robot will start moving in 3 seconds...")
    time.sleep(3)
    
    print(f"[UAT] DRIVE START! (500 steps)")
    for i in range(500):
        robot.set_joint_velocities(target_velocities)
        world.step(render=True)
        
        if i % 100 == 0:
            speed = np.linalg.norm(robot.get_linear_velocity())
            print(f"[UAT] Speed: {speed:.2f} m/s")

    print("\n[UAT] DRIVE COMPLETE. Keeping window open for 10 seconds for you to inspect.")
    for _ in range(100):
        simulation_app.update()
        time.sleep(0.1)

    simulation_app.close()

if __name__ == "__main__":
    run_proof()
