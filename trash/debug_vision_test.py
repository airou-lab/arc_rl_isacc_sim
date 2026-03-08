
import os
import numpy as np
import cv2
from isaacsim import SimulationApp

# Initialize simulation app
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.objects import DynamicCuboid
from omni.isaac.sensor import Camera
import omni.usd

def test_vision():
    usd_path = "/home/arika/Documents/arcpro/arcpro_system/src/examples/ARCPro_RL/arc_rl_isacc_sim/isacc_sim_usd/World0.usd"
    omni.usd.get_context().open_stage(usd_path)
    
    for _ in range(100):
        simulation_app.update()

    world = World()
    world.reset()

    robot_path = "/World/F1Tenth"
    # Get robot world position
    stage = omni.usd.get_context().get_stage()
    chassis_prim = stage.GetPrimAtPath(f"{robot_path}/Rigid_Bodies/Chassis")
    from pxr import UsdGeom
    chassis_xform = UsdGeom.Xformable(chassis_prim)
    chassis_pos = chassis_xform.ComputeLocalToWorldTransform(0).ExtractTranslation()
    chassis_ori = chassis_xform.ComputeLocalToWorldTransform(0).ExtractRotationQuat()
    
    print(f"Robot Position: {chassis_pos}")

    # Spawn a RED cube in front of the robot
    # Assuming forward is X
    cube_pos = chassis_pos + np.array([1.0, 0.0, 0.5]) 
    red_cube = DynamicCuboid(
        prim_path="/World/DebugCube",
        name="debug_cube",
        position=cube_pos,
        size=0.5,
        color=np.array([1.0, 0.0, 0.0]) # Red
    )
    
    # Try Onboard Camera
    cam_path = f"{robot_path}/Rigid_Bodies/Chassis/Camera_Left"
    camera = Camera(prim_path=cam_path, resolution=(160, 90))
    camera.initialize()
    
    print(f"[Debug] Vision test with Red Cube at {cube_pos}")
    
    for i in range(50):
        world.step(render=True)

    rgba = camera.get_rgba()
    if rgba is not None:
        rgb = rgba[:, :, :3]
        mean_val = np.mean(rgb)
        print(f"Vision Test Mean: {mean_val:.2f}")
        cv2.imwrite("vision_test_cube.png", cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
        print("Saved vision_test_cube.png")
    else:
        print("Capture failed.")

    simulation_app.close()

if __name__ == "__main__":
    test_vision()
