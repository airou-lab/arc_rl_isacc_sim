import numpy as np
import time
from isaac_direct_env import IsaacDirectEnv

def run_failure_test():
    print("[UAT] Starting Failure Detection Test...")
    env = IsaacDirectEnv(headless=True)
    
    try:
        # 1. Test Fall Detection
        print("\n[UAT] Testing Fall Detection (Z < -0.5)...")
        # Use new options to teleport during reset
        options = {"initial_state": {"position": np.array([0.0, 0.0, -1.0])}}
        obs, info = env.reset(options=options)
        
        obs, reward, terminated, truncated, info = env.step([0.0, 0.0])
        print(f"[UAT] Result: terminated={terminated}, Status={info.get('status')}")
        
        if terminated and info.get('status') == "Fallen":
            print("[UAT] Fall Detection: PASS")
        else:
            print("[UAT] Fall Detection: FAIL")

        # 2. Test Flip Detection
        print("\n[UAT] Testing Flip Detection (Roll > 45)...")
        # Teleport with 90 deg roll around X axis
        options = {"initial_state": {"position": np.array([0, 0, 0.1]), "orientation": np.array([0.707, 0.707, 0, 0])}}
        obs, info = env.reset(options=options)
        
        obs, reward, terminated, truncated, info = env.step([0.0, 0.0])
        print(f"[UAT] Result: terminated={terminated}, Status={info.get('status')}")
        
        if terminated and info.get('status') == "Flipped":
            print("[UAT] Flip Detection: PASS")
        else:
            print("[UAT] Flip Detection: FAIL")

    except Exception as e:
        print(f"[UAT] ERROR: {e}")
    finally:
        env.close()

if __name__ == "__main__":
    run_failure_test()
