
import os
import sys
import numpy as np
import torch
from sb3_contrib import RecurrentPPO

# Add policy directory to path for SB3 custom policy loading
POLICY_DIR = "/home/arika/Documents/arcpro/arcpro_system/src/examples/ARCPro_RL/arc_rl_isacc_policy"
if POLICY_DIR not in sys.path:
    sys.path.append(POLICY_DIR)

from isaac_direct_env import IsaacDirectEnv

# Path to your trained model
MODEL_PATH = "/home/arika/Documents/arcpro/arcpro_system/src/examples/ARCPro_RL/arc_rl_isacc_policy/models/isaac_hppo_20260223_210131/final_model.zip"

def run_inference_forced():
    print(f"[Inference Forced] Loading model from: {MODEL_PATH}")
    print("[Inference Forced] TESTING MODE: Adding +0.5 to AI Throttle to verify physics response.")
    
    # 1. Initialize the Environment
    env = IsaacDirectEnv(headless=False)
    
    try:
        # 2. Load the SB3 RecurrentPPO Model
        model = RecurrentPPO.load(MODEL_PATH)
        print("[Inference Forced] Model loaded successfully.")

        # 3. Inference Loop
        obs, info = env.reset()
        
        # Initialize LSTM hidden states
        lstm_states = None
        episode_start_masks = np.ones((1,), dtype=bool) 

        print("[Inference Forced] Starting AI Drive (FORCED). Press Ctrl+C to stop.")
        
        step_count = 0
        max_total_steps = 50 # Quick test
        total_steps = 0
        
        while total_steps < max_total_steps:
            total_steps += 1
            
            # --- Preprocessing: Normalize Image [0, 255] -> [0, 1] ---
            obs['image'] = obs['image'].astype(np.float32) / 255.0

            # 4. Predict Action
            action, lstm_states = model.predict(
                obs, 
                state=lstm_states, 
                episode_start=episode_start_masks, 
                deterministic=True
            )

            # 5. Execute Action with FORCE
            action_to_send = np.array(action).flatten()
            
            # FORCE: Override throttle to at least 0.5
            original_thr = action_to_send[1]
            action_to_send[1] = max(0.5, original_thr) 
            
            obs, reward, terminated, truncated, info = env.step(action_to_send)
            
            # 6. Telemetry
            if step_count % 10 == 0:
                print(f"[Forced] Step {step_count} | AI Steer: {action_to_send[0]:.2f} | AI Thr: {original_thr:.2f} -> FORCED: {action_to_send[1]:.2f} | Speed: {info.get('speed', 0):.2f} m/s")

            episode_start_masks = np.array([terminated or truncated])
            
            step_count += 1
            if terminated or truncated:
                print(f"[Inference] Episode finished. Status: {info.get('status')}")
                obs, info = env.reset()
                lstm_states = None
                episode_start_masks = np.ones((1,), dtype=bool)
                step_count = 0

    except KeyboardInterrupt:
        print("\n[Inference] Stopped by user.")
    except Exception as e:
        print(f"[Inference] ERROR: {e}")
    finally:
        print("[Inference] Closing Environment...")
        env.close()

if __name__ == "__main__":
    run_inference_forced()
