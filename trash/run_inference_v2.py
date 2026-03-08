
import os
import sys
import numpy as np

# 1. Initialize Isaac Sim FIRST (before Torch)
from isaacsim import SimulationApp
CONFIG = {"headless": False}
simulation_app = SimulationApp(CONFIG)

# 2. Now import Torch and other ML libs
import torch
from sb3_contrib import RecurrentPPO

# Add policy directory
POLICY_DIR = "/home/arika/Documents/arcpro/arcpro_system/src/examples/ARCPro_RL/arc_rl_isacc_policy"
if POLICY_DIR not in sys.path:
    sys.path.append(POLICY_DIR)

from isaac_direct_env import IsaacDirectEnv

# Path to your trained model
MODEL_PATH = "/home/arika/Documents/arcpro/arcpro_system/src/examples/ARCPro_RL/arc_rl_isacc_policy/models/isaac_hppo_20260223_210131/final_model.zip"

def run_inference():
    print(f"[Inference V2] Loading model from: {MODEL_PATH}")
    
    # Initialize the Environment (using existing SimulationApp)
    env = IsaacDirectEnv(headless=False, simulation_app=simulation_app)
    
    try:
        # Load the model
        model = RecurrentPPO.load(MODEL_PATH, env=env)
        print("[Inference V2] Model loaded successfully.")

        obs, info = env.reset()
        lstm_states = None
        episode_start = np.ones((1,), dtype=bool) 

        print("[Inference V2] Starting AI Drive. Press Ctrl+C to stop.")
        
        total_steps = 0
        max_steps = 500
        
        while total_steps < max_steps:
            # Predict
            action, lstm_states = model.predict(
                obs, 
                state=lstm_states, 
                episode_start=episode_start, 
                deterministic=True
            )

            # Execute
            action_to_send = np.array(action).flatten()
            obs, reward, terminated, truncated, info = env.step(action_to_send)
            
            # Telemetry
            if total_steps % 10 == 0:
                img_mean = np.mean(obs['image'])
                print(f"[Inference V2] Step {total_steps} | AI: Steer={action_to_send[0]:.2f}, Thr={action_to_send[1]:.2f} | Speed: {info.get('speed', 0):.2f} m/s | Img Mean: {img_mean:.1f}")

            episode_start = np.array([terminated or truncated])
            
            total_steps += 1
            if terminated or truncated:
                print(f"[Inference V2] Episode finished. Status: {info.get('status')}")
                obs, info = env.reset()
                lstm_states = None
                episode_start = np.ones((1,), dtype=bool)

    except KeyboardInterrupt:
        print("\n[Inference V2] Stopped by user.")
    except Exception as e:
        print(f"[Inference V2] ERROR: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("[Inference V2] Closing Environment...")
        env.close()

if __name__ == "__main__":
    run_inference()
