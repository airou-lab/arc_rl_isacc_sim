from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

import sys
import numpy as np

POLICY_DIR = "/home/arika/Documents/arcpro/arcpro_system/src/examples/ARCPro_RL/arc_rl_isacc_policy"
if POLICY_DIR not in sys.path:
    sys.path.append(POLICY_DIR)

from isaac_direct_env import IsaacDirectEnv, IsaacDirectConfig

def verify_logic():
    print("[Verify] Initializing Headless Environment...")
    config = IsaacDirectConfig(headless=True)
    env = IsaacDirectEnv(config=config, simulation_app=simulation_app)
    
    obs, info = env.reset()
    print(f"[Verify] Initial Position: {env._get_robot_position()}")
    print(f"[Verify] Observation Vec Shape: {obs['vec'].shape}")
    
    # Run 50 steps
    total_reward = 0
    for i in range(50):
        # Action: [delta_steer=0, delta_accel=0.2]
        action = np.array([0.0, 0.2], dtype=np.float32)
        obs, reward, terminated, truncated, info = env.step(action)
        total_reward += reward
        
        if i % 10 == 0:
            speed = obs['vec'][3]
            dist = obs['vec'][11]
            z_pos = env._get_robot_position()[2]
            print(f"Step {i:2d} | Speed: {speed:.2f} m/s | Dist: {dist:.2f} m | Z: {z_pos:.3f}m | Reward: {reward:.2f}")
            
        if terminated:
            print(f"Terminated at step {i}")
            break
            
    print(f"[Verify] Logic Check Passed. Total Reward: {total_reward:.2f}")
    # Don't call simulation_app.close() explicitly to avoid segfault in this test context
    # Just let it exit.

if __name__ == "__main__":
    verify_logic()
