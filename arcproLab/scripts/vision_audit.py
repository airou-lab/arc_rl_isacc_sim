import argparse
from isaaclab.app import AppLauncher

# 1. Setup Launcher
parser = argparse.ArgumentParser()
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

# 2. NOW import the rest
import os
import sys
import torch
import numpy as np
from PIL import Image

# Add current and parent dir to path
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(SCRIPT_DIR)
sys.path.append(os.path.abspath(os.path.join(SCRIPT_DIR, "..")))

from arcpro_env_cfg import ARCProEnvCfg
from isaaclab.envs import ManagerBasedRLEnv

def main():
    # Use a minimal config for verification
    env_cfg = ARCProEnvCfg()
    env_cfg.scene.num_envs = 1 
    env_cfg.sim.device = "cuda:0"
    
    # Create the environment
    env = ManagerBasedRLEnv(cfg=env_cfg)
    
    # Reset to get the first observation
    obs, _ = env.reset()
    
    print(f"Observation keys: {obs.keys()}")
    if "visual" in obs:
        if isinstance(obs["visual"], dict):
            print(f"Visual group keys: {obs['visual'].keys()}")
            img_tensor = obs["visual"]["tiled_camera"]
        else:
            print(f"Visual group is a tensor of shape: {obs['visual'].shape}")
            img_tensor = obs["visual"]
    else:
        print("CRITICAL: 'visual' group not found in obs!")
        env.close()
        simulation_app.close()
        return

    # Convert to numpy (B, H, W, C)
    img_np = img_tensor[0].cpu().numpy()
    print(f"Extracted image shape: {img_np.shape}, Dtype: {img_np.dtype}")
    
    # If it's (C, H, W), transpose to (H, W, C) for PIL
    if img_np.shape[0] == 3:
        img_np = img_np.transpose(1, 2, 0)
        
    # Ensure it's uint8
    if img_np.dtype != np.uint8:
        if img_np.max() <= 1.0:
            img_np = (img_np * 255).astype(np.uint8)
        else:
            img_np = img_np.astype(np.uint8)
            
    # Save the image
    Image.fromarray(img_np).save("vision_audit.png")
    print(f"SUCCESS: Vision audit frame saved to vision_audit.png. Mean: {img_np.mean()}")
    
    env.close()
    simulation_app.close()

if __name__ == "__main__":
    main()
