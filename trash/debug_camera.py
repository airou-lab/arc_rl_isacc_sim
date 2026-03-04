from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

import omni.usd
import numpy as np
from PIL import Image
from isaac_direct_env import IsaacDirectEnv

def save_camera_frame():
    print("[Diagnostic] Initializing environment to capture a frame...")
    env = IsaacDirectEnv(headless=True)
    
    try:
        obs, info = env.reset()
        image_data = obs["image"] 
        
        # Save using PIL
        img = Image.fromarray(image_data)
        img.save("camera_debug.png")
        print(f"[Diagnostic] Frame saved to 'camera_debug.png'. Shape: {image_data.shape}")
        
        print(f"[Diagnostic] Pixel Mean: {np.mean(image_data):.2f}, Max: {np.max(image_data)}")

    except Exception as e:
        print(f"[Diagnostic] ERROR: {e}")
    finally:
        env.close()

if __name__ == "__main__":
    save_camera_frame()
