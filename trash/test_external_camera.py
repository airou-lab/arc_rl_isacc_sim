
import os
import numpy as np
import cv2
from isaacsim import SimulationApp

# Initialize simulation app
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.sensor import Camera
import omni.usd

def test_external():
    usd_path = "/home/arika/Documents/arcpro/arcpro_system/src/examples/ARCPro_RL/arc_rl_isacc_sim/isacc_sim_usd/World0.usd"
    omni.usd.get_context().open_stage(usd_path)
    
    for _ in range(100):
        simulation_app.update()

    world = World()
    world.reset()

    # Create an EXTERNAL camera at a high vantage point
    cam_path = "/World/ExternalVantage"
    camera = Camera(
        prim_path=cam_path,
        resolution=(160, 90),
        position=np.array([5.0, 5.0, 5.0]),
        orientation=np.array([0.5, 0.5, 0.5, 0.5]) # Pointing towards origin
    )
    camera.initialize()
    
    print(f"[Debug] External camera spawned at [5, 5, 5]")
    
    for i in range(50):
        world.step(render=True)

    rgba = camera.get_rgba()
    if rgba is not None:
        rgb = rgba[:, :, :3]
        mean_val = np.mean(rgb)
        print(f"External Cam Mean: {mean_val:.2f}")
        cv2.imwrite("debug_external_vantage.png", cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
        print("Saved debug_external_vantage.png")
    else:
        print("Capture failed.")

    simulation_app.close()

if __name__ == "__main__":
    test_external()
