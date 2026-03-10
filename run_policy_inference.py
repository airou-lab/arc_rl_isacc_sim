import os
import sys
import numpy as np
import time

# 1. Setup paths for custom policies first
POLICY_DIR = "/home/arika/Documents/arcpro/arcpro_system/src/examples/ARCPro_RL/arc_rl_isacc_policy"
if POLICY_DIR not in sys.path:
    sys.path.append(POLICY_DIR)

# 2. Initialize Isaac Sim BEFORE any ML imports
print("[Inference] Initializing SimulationApp...")
from isaacsim import SimulationApp
# Headless enabled for remote execution
simulation_app = SimulationApp({"headless": True})

# 3. Imports after SimulationApp is ready
print("[Inference] Loading libraries...")
try:
    import cv2
    from isaac_direct_env import IsaacDirectEnv, IsaacDirectConfig
    print("[Inference] Libraries loaded successfully.")
except Exception as e:
    print(f"[Inference] ERROR during library loading: {e}")
    simulation_app.close()
    sys.exit(1)

def run_policy():
    print("\n--- Phase 2: Autonomous Scouting (Manual Drive) ---")
    
    print("[Inference] Initializing IsaacDirectEnv...")
    try:
        config = IsaacDirectConfig(headless=True)
        env = IsaacDirectEnv(config=config, simulation_app=simulation_app)
        print("[Inference] Env initialized.")
    except Exception as e:
        print(f"[Inference] ERROR initializing env: {e}")
        simulation_app.close()
        return
    
    print("[Inference] Resetting environment...")
    try:
        obs, info = env.reset()
        print("[Inference] Environment reset complete.")
    except Exception as e:
        print(f"[Inference] ERROR during reset: {e}")
        env.close()
        simulation_app.close()
        return
    
    steps = 0
    print("[Inference] Starting Autonomous Scouting loop (Manual Forward)...")
    try:
        # Run for 300 steps to scout
        for i in range(300):
            # 4. Manual Action [steer, throttle, brake]
            # Slow forward to avoid collisions
            action = np.array([0.0, 0.3, 0.0], dtype=np.float32) 
            
            # 6. Apply Action
            obs, reward, terminated, truncated, info = env.step(action)
            steps += 1
            
            if i % 20 == 0:
                pos = env._get_robot_position()
                speed = np.linalg.norm(env._get_robot_velocity())
                print(f"Step {i:4d} | Pos: {pos} | Speed: {speed:5.2f} m/s")

            # Periodic visual capture
            if i % 50 == 0:
                frame = obs["image"]
                cv2.imwrite(f"scout_frame_{i}.png", cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))
                print(f"  [Scout] Saved frame at step {i}")

            if terminated or truncated:
                print(f"Scout Terminated at step {i}. Reason: {info.get('reset_reason', 'timeout')}")
                break

    except Exception as e:
        print(f"[Inference] Loop Error: {e}")
    finally:
        print(f"\nFinal Summary: {steps} steps completed. Cleaning up...")
        # Safer shutdown order
        try:
            env.close()
            simulation_app.close()
        except:
            pass

if __name__ == "__main__":
    run_policy()
