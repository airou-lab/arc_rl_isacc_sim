from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

import sys
import numpy as np
import time

POLICY_DIR = "/home/arika/Documents/arcpro/arcpro_system/src/examples/ARCPro_RL/arc_rl_isacc_policy"
if POLICY_DIR not in sys.path:
    sys.path.append(POLICY_DIR)

from isaac_direct_env import IsaacDirectEnv, IsaacDirectConfig

def smoke_test():
    print("[SmokeTest] Initializing Environment...")
    config = IsaacDirectConfig(headless=False)
    env = IsaacDirectEnv(config=config, simulation_app=simulation_app)
    
    obs, info = env.reset()
    print(f"[SmokeTest] Reset Complete. Initial Obs Keys: {obs.keys()}")
    
    # 10 test steps with forward throttle
    for i in range(10):
        # Action: [delta_steer=0, delta_accel=0.1]
        action = np.array([0.0, 0.1], dtype=np.float32)
        obs, reward, terminated, truncated, info = env.step(action)
        
        speed = obs['vec'][3]
        dist = obs['vec'][11]
        print(f"Step {i} | Speed: {speed:.2f} m/s | Total Dist: {dist:.2f} m | Reward: {reward:.2f}")
        
        if terminated:
            print("Terminated early.")
            break
            
    print("[SmokeTest] Success. Simulation loop complete.")
    # No close() to avoid exit crash

if __name__ == "__main__":
    smoke_test()
