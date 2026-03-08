
import os
import numpy as np
import cv2
from isaacsim import SimulationApp

# Initialize simulation app
simulation_app = SimulationApp({"headless": True})

from omni.isaac.core import World
from omni.isaac.core.articulations import Articulation
from omni.isaac.sensor import Camera
import omni.usd

def test_air():
    usd_path = "/home/arika/Documents/arcpro/arcpro_system/src/examples/ARCPro_RL/arc_rl_isacc_sim/isacc_sim_usd/World0.usd"
    omni.usd.get_context().open_stage(usd_path)
    
    for _ in range(100):
        simulation_app.update()

    world = World()
    world.reset()

    robot_path = "/World/F1Tenth"
    robot = Articulation(robot_path)
    world.scene.add(robot)
    world.reset()
    
    # Spawn robot 10 meters in the air
    robot.set_world_pose(position=np.array([0.0, 0.0, 10.0]))
    
    # Try Onboard Camera
    cam_path = f"{robot_path}/Rigid_Bodies/Chassis/Camera_Left"
    camera = Camera(prim_path=cam_path, resolution=(160, 90))
    camera.initialize()
    
    print(f"[Debug] Robot spawned in AIR at Z=10.0")
    
    for i in range(20):
        world.step(render=True)

    rgba = camera.get_rgba()
    if rgba is not None:
        rgb = rgba[:, :, :3]
        mean_val = np.mean(rgb)
        print(f"Air Test Mean: {mean_val:.2f}")
        cv2.imwrite("debug_air_view.png", cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
        print("Saved debug_air_view.png")
    else:
        print("Capture failed.")

    simulation_app.close()

if __name__ == "__main__":
    test_air()
