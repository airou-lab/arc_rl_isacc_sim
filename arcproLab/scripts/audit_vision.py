
import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Audit the camera output.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import torch
import os
import sys
import numpy as np
import cv2

# Add arcproLab to path
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
ARCPRO_LAB_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, ".."))
if ARCPRO_LAB_DIR not in sys.path:
    sys.path.append(ARCPRO_LAB_DIR)

from arcpro_env_cfg import ARCProEnvCfg
from isaaclab.envs import ManagerBasedRLEnv

def main():
    env_cfg = ARCProEnvCfg()
    env_cfg.scene.num_envs = 1
    # Ensure cameras are enabled
    env_cfg.enable_cameras = True
    env = ManagerBasedRLEnv(cfg=env_cfg)
    
    print("\n" + "="*60)
    print("CAMERA VISION AUDIT")
    print("Goal: Capture and save camera frames to verify visibility.")
    print("="*60)
    
    obs_dict, _ = env.reset()
    
    os.makedirs("debug_frames", exist_ok=True)
    
    # Action: [steer, drive]
    # Steer slightly to the right (positive steer) and drive forward
    action = torch.tensor([[0.5, 0.2]], device=env.device)
    
    for i in range(200):
        obs_dict, rewards, terminated, truncated, info = env.step(action)
        
        # Capture frame at step 0, 50, 100, 150, 200
        if i % 50 == 0:
            if "tiled_camera" in env.scene.sensors:
                # Get RAW unnormalized image directly from sensor
                img = env.scene.sensors["tiled_camera"].data.output["rgb"].cpu().numpy()[0] # (224, 224, 3)
                
                print(f"[DEBUG] Frame {i}: shape={img.shape}, dtype={img.dtype}, min={np.min(img)}, max={np.max(img)}, mean={np.mean(img):.2f}")
                
                # It's uint8 [0, 255]
                img_bgr = cv2.cvtColor(img.astype(np.uint8), cv2.COLOR_RGB2BGR)
                
                cv2.imwrite(f"debug_frames/frame_{i}.png", img_bgr)
                print(f"[DEBUG] Saved camera frame {i} to debug_frames/frame_{i}.png")
                
                if np.mean(img) < 1.0:
                    print(f"!!! WARNING: Frame {i} appears to be BLACK.")
            else:
                print("!!! ERROR: 'tiled_camera' sensor not found.")

        if terminated[0]:
            print(f"Episode terminated at step {i}.")
            env.reset()

    simulation_app.close()

if __name__ == "__main__":
    main()
