
import os
import numpy as np
from isaacsim import SimulationApp

# Initialize simulation app
simulation_app = SimulationApp({"headless": True})

from omni.isaac.core import World
from omni.isaac.core.articulations import Articulation
from omni.isaac.sensor import Camera
import omni.usd

def check_camera():
    usd_path = "/home/arika/Documents/arcpro/arcpro_system/src/examples/ARCPro_RL/arc_rl_isacc_sim/isacc_sim_usd/World0.usd"
    omni.usd.get_context().open_stage(usd_path)
    simulation_app.update()

    world = World()
    world.reset()

    robot_path = "/World/F1Tenth"
    camera_path = f"{robot_path}/Rigid_Bodies/Chassis/Camera_Left"
    camera = Camera(prim_path=camera_path, resolution=(160, 90))
    camera.initialize()
    
    # Take a few steps to let rendering warm up
    for _ in range(10):
        world.step(render=True)

    rgba = camera.get_rgba()
    print(f"Camera Output Shape: {rgba.shape}")
    print(f"Camera Output Dtype: {rgba.dtype}")
    print(f"Camera Output Min: {np.min(rgba)}")
    print(f"Camera Output Max: {np.max(rgba)}")
    
    rgb = rgba[:, :, :3]
    print(f"RGB Part Shape: {rgb.shape}")
    print(f"RGB Part Dtype: {rgb.dtype}")

    simulation_app.close()

if __name__ == "__main__":
    check_camera()
