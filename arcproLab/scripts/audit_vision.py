
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
    
    # Action: [steer, throttle, brake]
    # Small forward nudge to see if lines move
    action = torch.tensor([[0.0, 0.2, 0.0]], device=env.device)
    
    for i in range(200):
        obs_dict, rewards, terminated, truncated, info = env.step(action)
        
        # Capture frame at step 0, 50, 100, 150, 200
        if i % 50 == 0:
            if "visual" in obs_dict:
                img = obs_dict["visual"].cpu().numpy()[0] # (224, 224, 3)
                # It's float32 [0, 1], convert to uint8
                img_uint8 = (np.clip(img, 0, 1) * 255).astype(np.uint8)
                # RGB to BGR for cv2
                img_bgr = cv2.cvtColor(img_uint8, cv2.COLOR_RGB2BGR)
                
                fname = f"debug_frames/frame_{i}.png"
                cv2.imwrite(fname, img_bgr)
                print(f"[DEBUG] Saved camera frame {i} to {fname}")
                
                # Check for black frames
                if np.mean(img) < 0.01:
                    print(f"!!! WARNING: Frame {i} appears to be BLACK.")
            else:
                print("!!! ERROR: 'visual' observations not found in obs_dict.")

        if terminated[0]:
            print(f"Episode terminated at step {i}.")
            env.reset()

    simulation_app.close()

if __name__ == "__main__":
    main()
