import os
import sys
import numpy as np
import time

MODEL_PATH = "/home/arika/Documents/arcpro/arcpro_system/src/examples/ARCPro_RL/arc_rl_isacc_policy/models/isaac_hppo_20260223_210131/final_model.zip"

def run_inference():
    print("[Inference] Initializing Isaac Sim (with GUI for Freecam)...")
    from isaacsim import SimulationApp
    simulation_app = SimulationApp({"headless": False})

    import torch
    from sb3_contrib import RecurrentPPO

    POLICY_DIR = "/home/arika/Documents/arcpro/arcpro_system/src/examples/ARCPro_RL/arc_rl_isacc_policy"
    if POLICY_DIR not in sys.path:
        sys.path.append(POLICY_DIR)

    from isaac_direct_env import IsaacDirectEnv, IsaacDirectConfig

    print(f"[Inference] Loading model from: {MODEL_PATH}")
    
    config = IsaacDirectConfig(headless=False)
    config.max_episode_steps = 2000 
    
    env = IsaacDirectEnv(config=config, simulation_app=simulation_app)
    
    try:
        if not os.path.exists(MODEL_PATH):
            print(f"[Inference] ERROR: Model file not found at {MODEL_PATH}")
            return

        model = RecurrentPPO.load(MODEL_PATH)
        print("[Inference] Model loaded successfully.")

        obs, info = env.reset()
        
        lstm_states = None
        episode_start_masks = np.ones((1,), dtype=bool) 

        print("[Inference] Starting AI Drive. Press Ctrl+C to stop.")
        print("[Inference] FREECAM MODE: Use mouse to navigate the viewport.")
        
        total_steps = 0
        laps_attempted = 0
        max_laps = 3

        while simulation_app.is_running() and laps_attempted < max_laps:
            if np.any(np.isnan(obs['vec'])):
                print(f"[Inference] CRITICAL ERROR: Input Telemetry contains NaN! {obs['vec']}")
                break
            
            try:
                action, lstm_states = model.predict(
                    obs, 
                    state=lstm_states, 
                    episode_start=episode_start_masks, 
                    deterministic=True
                )
            except Exception as e:
                print(f"[Inference] Policy Crash: {e}")
                break

            action_to_send = np.array(action).flatten()
            obs, reward, terminated, truncated, info = env.step(action_to_send)
            
            dist = obs['vec'][11] 
            speed = obs['vec'][3]

            if total_steps % 20 == 0:
                print(f"[Inference] Step {total_steps:4d} | Speed: {speed:4.2f} m/s | Dist: {dist:6.2f} m")

            episode_start_masks = np.array([terminated or truncated])
            total_steps += 1
            
            if terminated or truncated:
                laps_attempted += 1
                print(f"[Inference] Episode finished. Final Distance: {dist:.2f} meters. (Attempt {laps_attempted}/{max_laps})")
                if dist > 80.0:
                    print("[Inference] SUCCESS: Likely completed a full lap!")
                    print("[Inference] PAUSING for 30 seconds for visual verification...")
                    time.sleep(30)
                    break
                else:
                    if laps_attempted < max_laps:
                        print("[Inference] Resetting...")
                        obs, info = env.reset()
                        lstm_states = None
                        episode_start_masks = np.ones((1,), dtype=bool)
                        total_steps = 0
                    else:
                        print("[Inference] Reached max attempts. Exiting.")
                        break

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
