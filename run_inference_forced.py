import os
import sys
import numpy as np
import torch
from sb3_contrib import RecurrentPPO

# Add policy directory to path
POLICY_DIR = "/home/arika/Documents/arcpro/arcpro_system/src/examples/ARCPro_RL/arc_rl_isacc_policy"
if POLICY_DIR not in sys.path:
    sys.path.append(POLICY_DIR)

from isaac_direct_env import IsaacDirectEnv

# Path to your trained model
MODEL_PATH = "/home/arika/Documents/arcpro/arcpro_system/src/examples/ARCPro_RL/arc_rl_isacc_policy/models/isaac_hppo_20260223_210131/final_model.zip"

def run_forced_inference(force_throttle=True):
    print(f"[Inference] Loading model from: {MODEL_PATH}")
    print(f"[Inference] FORCED THROTTLE: {force_throttle}")
    
    # 1. Initialize the Environment
    env = IsaacDirectEnv(headless=False)
    
    try:
        # 2. Load the SB3 RecurrentPPO Model
        model = RecurrentPPO.load(MODEL_PATH)
        print("[Inference] Model loaded successfully.")

        # 3. Inference Loop
        obs, info = env.reset()
        lstm_states = None
        episode_start_masks = np.ones((1,), dtype=bool) 

        print("[Inference] Starting AI Drive with Manual Speed Override. Press Ctrl+C to stop.")
        
        step_count = 0
        while step_count < 1000:
            # 4. Predict Action
            action, lstm_states = model.predict(
                obs, 
                state=lstm_states, 
                episode_start=episode_start_masks, 
                deterministic=True
            )

            # 5. Prepare Action
            action_to_send = np.array(action).flatten()
            
            if force_throttle:
                action_to_send[1] = 0.3 # Constant forward push
                action_to_send[2] = 0.0 # No brakes
            
            # 6. Execute Action
            obs, reward, terminated, truncated, info = env.step(action_to_send)
            
            # Telemetry
            if step_count % 10 == 0:
                print(f"[Inference] Step {step_count} | AI Steer: {action_to_send[0]:.2f} | Speed: {info.get('speed', 0):.2f} m/s | Status: {info.get('status')}")

            # Update masks for LSTM
            episode_start_masks = np.array([terminated or truncated])
            
            step_count += 1
            if terminated or truncated:
                print(f"[Inference] Resetting... Status: {info.get('status')}")
                obs, info = env.reset()
                lstm_states = None
                episode_start_masks = np.ones((1,), dtype=bool)

    except KeyboardInterrupt:
        print("\n[Inference] Stopped by user.")
    except Exception as e:
        print(f"[Inference] ERROR: {e}")
    finally:
        print("[Inference] Closing Environment...")
        env.close()

if __name__ == "__main__":
    run_forced_inference(force_throttle=True)
