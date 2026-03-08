
import os
import numpy as np
import cv2
from isaacsim import SimulationApp

# Initialize simulation app
simulation_app = SimulationApp({"headless": True})

from isaac_direct_env import IsaacDirectEnv

def capture_image():
    print("[Debug] Launching env to capture camera frame...")
    env = IsaacDirectEnv(headless=True)
    obs, info = env.reset()
    
    # Take 10 steps to ensure rendering is active
    for _ in range(10):
        obs, _, _, _, _ = env.step(np.array([0.0, 0.0, 0.0]))
    
    image = obs["image"]
    print(f"Captured image shape: {image.shape}")
    print(f"Image mean value: {np.mean(image)}")
    
    # Save as BGR for OpenCV
    save_path = "debug_camera_frame.png"
    cv2.imwrite(save_path, cv2.cvtColor(image, cv2.COLOR_RGB2BGR))
    print(f"Image saved to: {os.path.abspath(save_path)}")
    
    env.close()

if __name__ == "__main__":
    capture_image()
