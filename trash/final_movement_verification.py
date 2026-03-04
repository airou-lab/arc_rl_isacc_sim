
import os
import numpy as np
from isaacsim import SimulationApp

# Initialize simulation app (Headless for verification)
simulation_app = SimulationApp({"headless": True})

from isaac_direct_env import IsaacDirectEnv

def run_verification():
    print("\n[UAT] Starting Final Movement Verification...")
    env = IsaacDirectEnv(headless=True)
    obs, info = env.reset()
    
    # Action: [Steer, Throttle, Brake]
    # We send moderate throttle (0.6) to verify responsive movement
    action = np.array([0.0, 0.6, 0.0], dtype=np.float32)
    
    print(f"[UAT] Applying sustained action: {action}")
    
    steps = 100
    speeds = []
    
    for i in range(steps):
        obs, reward, terminated, truncated, info = env.step(action)
        speed = info.get("speed", 0.0)
        speeds.append(speed)
        
        if i % 20 == 0:
            print(f"[UAT] Step {i}: Speed = {speed:.2f} m/s")
            
    avg_speed = np.mean(speeds[50:]) # Average speed after ramp-up
    max_speed = np.max(speeds)
    
    print(f"\n--- Final Results ---")
    print(f"Max Speed: {max_speed:.2f} m/s")
    print(f"Average Speed (Steady State): {avg_speed:.2f} m/s")
    
    if max_speed > 2.0:
        print("\nVERIFICATION: SUCCESS. Robot is responsive and moving at high speed.")
    else:
        print("\nVERIFICATION: FAILURE. Robot is still moving too slowly.")

    env.close()

if __name__ == "__main__":
    run_verification()
