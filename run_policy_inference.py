import os
import sys
import numpy as np
import time

# 1. Initialize Isaac Sim BEFORE any ML imports
from isaacsim import SimulationApp
# Using headless for the first automated test run
simulation_app = SimulationApp({"headless": False})

# ... (omitted same part)

    # Initialize Environment
    config = IsaacDirectConfig(headless=False)

# ... (omitted same part)

    print("Starting Autonomous Drive (GUI Mode)...")
    try:
        # Run for 2000 steps for user observation
        for i in range(2000):
            # 4. Policy Inference
            # deterministic=True for testing/deployment
            action, lstm_states = model.predict(
                obs, 
                state=lstm_states, 
                episode_start=episode_starts, 
                deterministic=True
            )
            
            # 5. Simulated Latency (20ms delay as discussed)
            # This makes the 26-joint model handling more realistic
            time.sleep(0.02)
            
            # 6. Apply Action
            obs, reward, terminated, truncated, info = env.step(action)
            episode_starts = np.array([terminated or truncated])
            
            total_reward += reward
            steps += 1
            
            if i % 50 == 0:
                # Get linear velocity from the articulation
                speed = np.linalg.norm(env._robot_articulation.get_linear_velocity())
                img_mean = np.mean(obs["image"])
                print(f"Step {i:4d} | Speed: {speed:5.2f} m/s | Img Mean: {img_mean:6.1f} | Reward: {total_reward:8.2f}")

            if terminated or truncated:
                print(f"Episode Finished at step {i}. Reason: {info.get('reset_reason', 'collision/off-track')}")
                break

    except Exception as e:
        print(f"Inference Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print(f"\nFinal Summary: {steps} steps completed. Total Reward: {total_reward:.2f}")
        print("Closing environment...")
        env.close()
        simulation_app.close()

if __name__ == "__main__":
    run_policy()
