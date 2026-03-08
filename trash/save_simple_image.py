
import os
import numpy as np
import cv2
from isaacsim import SimulationApp

# Initialize simulation app
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.sensor import Camera
import omni.usd

def capture():
    usd_path = "/home/arika/Documents/arcpro/arcpro_system/src/examples/ARCPro_RL/arc_rl_isacc_sim/isacc_sim_usd/World0.usd"
    omni.usd.get_context().open_stage(usd_path)
    
    for _ in range(100):
        simulation_app.update()

    world = World()
    world.reset()

    robot_path = "/World/F1Tenth"
    cameras = ["Camera_Left", "Camera_Right", "Camera_Chase"]
    
    for cam_name in cameras:
        cam_path = f"{robot_path}/Rigid_Bodies/Chassis/{cam_name}"
        camera = Camera(prim_path=cam_path, resolution=(160, 90))
        camera.initialize()
        
        print(f"\n--- Testing {cam_name} ---")
        for i in range(20):
            world.step(render=True)

        rgba = camera.get_rgba()
        if rgba is not None:
            rgb = rgba[:, :, :3]
            mean_val = np.mean(rgb)
            print(f"[{cam_name}] Shape: {rgb.shape}, Mean: {mean_val:.2f}")
            
            save_path = f"debug_{cam_name.lower()}.png"
            cv2.imwrite(save_path, cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
            print(f"Saved to {save_path}")
        else:
            print(f"[{cam_name}] FAILED to capture.")

    simulation_app.close()

if __name__ == "__main__":
    capture()
