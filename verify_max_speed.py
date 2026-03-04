
import os
import numpy as np
from isaacsim import SimulationApp

# Initialize simulation app
simulation_app = SimulationApp({"headless": True})

from isaac_direct_env import IsaacDirectEnv

def test_max_speed():
    print("\n--- Starting Max Speed Verification (Full Throttle: 1.0) ---")
    env = IsaacDirectEnv(headless=True)
    obs, info = env.reset()
    
    # Action: [Steer, Throttle, Brake]
    # Full throttle, zero steering, zero brake
    action = np.array([0.0, 1.0, 0.0], dtype=np.float32)
    
    max_observed_speed = 0.0
    
    for i in range(100):
        obs, reward, terminated, truncated, info = env.step(action)
        speed = info.get("speed", 0.0)
        max_observed_speed = max(max_observed_speed, speed)
        
        if i % 20 == 0:
            print(f"Step {i}: Current Speed = {speed:.2f} m/s")
            
    print(f"\n--- Result ---")
    print(f"Max Speed Reached: {max_observed_speed:.2f} m/s")
    
    if max_observed_speed > 4.5:
        print("STATUS: High-speed performance verified (Target ~5.0 m/s).")
    else:
        print("STATUS: Speed restricted. Check wheel radius or friction.")

    env.close()

if __name__ == "__main__":
    test_max_speed()
