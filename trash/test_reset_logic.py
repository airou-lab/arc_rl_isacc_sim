
import os
import numpy as np
from isaacsim import SimulationApp

# Initialize simulation app
simulation_app = SimulationApp({"headless": True})

from isaac_direct_env import IsaacDirectEnv

def test_reset_logic():
    print("\n[UAT] Starting Failure & Reset Detection Test...")
    env = IsaacDirectEnv(headless=True)
    
    # --- Case 1: Fall Detection ---
    print("\n[UAT] Testing Fall Detection (Simulating Z < -0.5)...")
    env.reset()
    # Manually warp robot below the floor
    env.robot.set_world_pose(position=np.array([0.0, 0.0, -1.0]))
    obs, reward, terminated, truncated, info = env.step(np.array([0.0, 0.0, 0.0]))
    
    if terminated and info.get("status") == "Fallen":
        print("SUCCESS: Fall detected correctly.")
    else:
        print(f"FAILURE: Fall NOT detected. Status: {info.get('status')}, Terminated: {terminated}")

    # --- Case 2: Flip Detection ---
    print("\n[UAT] Testing Flip Detection (Simulating Pitch > 45 deg)...")
    env.reset()
    # Manually warp robot to be flipped (90 deg pitch)
    # Using a simple orientation that represents a flip
    flipped_ori = np.array([0.7071, 0.0, 0.7071, 0.0]) # 90 deg pitch
    env.robot.set_world_pose(orientation=flipped_ori)
    obs, reward, terminated, truncated, info = env.step(np.array([0.0, 0.0, 0.0]))
    
    if terminated and info.get("status") == "Flipped":
        print("SUCCESS: Flip detected correctly.")
    else:
        print(f"FAILURE: Flip NOT detected. Status: {info.get('status')}, Terminated: {terminated}")

    env.close()

if __name__ == "__main__":
    test_reset_logic()
