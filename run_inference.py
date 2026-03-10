import os
import sys
import numpy as np

# Path to your trained model
MODEL_PATH = "/home/arika/Documents/arcpro/arcpro_system/src/examples/ARCPro_RL/arc_rl_isacc_policy/models/isaac_hppo_20260223_210131/final_model.zip"

def run_inference():
    # 1. Initialize Isaac Sim FIRST (before Torch/ML libs)
    from isaacsim import SimulationApp
    simulation_app = SimulationApp({"headless": False})

    # 2. Now import ML libs (safely inside SimulationApp context)
    import torch
    from sb3_contrib import RecurrentPPO

    # Add policy directory
    POLICY_DIR = "/home/arika/Documents/arcpro/arcpro_system/src/examples/ARCPro_RL/arc_rl_isacc_policy"
    if POLICY_DIR not in sys.path:
        sys.path.append(POLICY_DIR)

    from isaac_direct_env import IsaacDirectEnv, IsaacDirectConfig

    print(f"[Inference] Loading model from: {MODEL_PATH}")
    
    # 3. Initialize Environment
    config = IsaacDirectConfig(headless=False)
    env = IsaacDirectEnv(config=config, simulation_app=simulation_app)
    
    try:
        # 4. Load the SB3 RecurrentPPO Model
        model = RecurrentPPO.load(MODEL_PATH)
        print("[Inference] Model loaded successfully.")

        # 5. Inference Loop
        obs, info = env.reset()
        
        lstm_states = None
        episode_start_masks = np.ones((1,), dtype=bool) 

        print("[Inference] Starting AI Drive. Press Ctrl+C to stop.")
        
        total_steps = 0
        max_total_steps = 1000
        
        while total_steps < max_total_steps:
            # --- DEBUG: Robust NaN Check ---
            if np.any(np.isnan(obs['vec'])):
                print(f"[Inference] CRITICAL ERROR: NaN detected in telemetry! Vec: {obs['vec']}")
                break
            
            # Predict Action
            action, lstm_states = model.predict(
                obs, 
                state=lstm_states, 
                episode_start=episode_start_masks, 
                deterministic=True
            )

            # Execute Action
            action_to_send = np.array(action).flatten()
            obs, reward, terminated, truncated, info = env.step(action_to_send)
            
            # Telemetry
            if total_steps % 10 == 0:
                print(f"[Inference] Step {total_steps} | AI: Steer={action_to_send[0]:.2f}, Thr={action_to_send[1]:.2f} | Speed: {info.get('speed', 0):.2f} m/s")

            episode_start_masks = np.array([terminated or truncated])
            
            total_steps += 1
            if terminated or truncated:
                print(f"[Inference] Episode finished. Status: {info.get('status')}")
                obs, info = env.reset()
                lstm_states = None
                episode_start_masks = np.ones((1,), dtype=bool)

    except KeyboardInterrupt:
        print("\n[Inference] Stopped by user.")
    except Exception as e:
        print(f"[Inference] ERROR: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("[Inference] Closing Environment...")
        env.close()
        simulation_app.close()

if __name__ == "__main__":
    run_inference()
