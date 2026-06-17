
import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
args_cli.enable_cameras = True

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import torch
import numpy as np
import cv2
import omni.isaac.core.utils.prims as prim_utils
from isaaclab.sensors import TiledCameraCfg, TiledCamera
import isaaclab.sim as sim_utils

def main():
    # 1. Create a simple scene with a cube
    prim_utils.create_prim("/World/Cube", "Cube", translation=(0, 0, 0.5), scale=(1, 1, 1))
    sim_utils.spawn_distant_light("/World/Light", intensity=3000.0)
    
    # 2. Create a TiledCamera at a fixed world position looking at the cube
    cam_cfg = TiledCameraCfg(
        prim_path="/World/TestCamera",
        update_period=0.0,
        spawn=sim_utils.PinholeCameraCfg(),
        width=224, height=224,
        data_types=["rgb"],
    )
    camera = TiledCamera(cam_cfg)
    
    # Set world pose: 5 meters away, looking at origin
    camera.set_world_poses(
        positions=torch.tensor([[5.0, 0.0, 1.0]]),
        quaternions=torch.tensor([[0.7071, 0.0, -0.7071, 0.0]]) # Looking along -X
    )
    
    # 3. Simulate and capture
    simulation_app.update()
    camera.update(dt=0.01)
    
    img = camera.data.output["rgb"].cpu().numpy()[0]
    img_uint8 = (np.clip(img, 0, 1) * 255).astype(np.uint8)
    cv2.imwrite("test_render.png", cv2.cvtColor(img_uint8, cv2.COLOR_RGB2BGR))
    
    print(f"Test render saved. Mean brightness: {img.mean():.4f}")
    
    simulation_app.close()

if __name__ == "__main__":
    main()
