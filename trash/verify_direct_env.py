import numpy as np
import time
from isaac_direct_env import IsaacDirectEnv

def run_verification():
    print("[Verification] Starting Aligned IsaacDirectEnv Verification...")
    
    # Initialize environment
    env = IsaacDirectEnv(headless=False)
    
    try:
        # 1. Test Reset
        print("[Verification] Testing Reset...")
        obs, info = env.reset()
        print(f"[Verification] Image shape: {obs['image'].shape}")
        print(f"[Verification] Vector shape: {obs['vec'].shape}")
        
        # 2. Run Random Actions for 50 steps
        print("[Verification] Running 50 steps with [steer, throttle, brake] actions...")
        for i in range(50):
            # Sample random action: [steer (-1 to 1), throttle (0 to 1), brake (0 to 1)]
            action = env.action_space.sample()
            # Force forward movement
            action[1] = 0.5 # 50% throttle
            action[2] = 0.0 # 0% brake
            
            obs, reward, terminated, truncated, info = env.step(action)
            
            if terminated or truncated:
                print(f"[Verification] Environment reset triggered (Status: {info.get('status')})")
                env.reset()
        
        print("[Verification] 50 steps completed successfully.")
        
    except Exception as e:
        print(f"[Verification] ERROR: {e}")
    finally:
        env.close()

if __name__ == "__main__":
    run_verification()
