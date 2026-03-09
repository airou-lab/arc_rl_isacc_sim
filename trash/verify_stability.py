import os
import sys
import numpy as np
import time

# Add the policy directory so we can import the environment
POLICY_DIR = "/home/arika/Documents/arcpro/arcpro_system/src/examples/ARCPro_RL/arc_rl_isacc_policy"
if POLICY_DIR not in sys.path:
    sys.path.append(POLICY_DIR)

from isaac_direct_env import IsaacDirectEnv, IsaacDirectConfig

def verify_stability():
    print("\n--- Phase 1.3: Physics & Visual Stability Verification ---")
    
    # 1. Initialize Isaac Sim
    from isaacsim import SimulationApp
    # Running with GUI so we can see the "explosions" or "white-out" if they happen
    simulation_app = SimulationApp({"headless": True})

    # 2. Configure Environment for Testing
    config = IsaacDirectConfig(headless=True)
    # We'll use the existing spawn as a base for our 5 test points
    base_x, base_y, base_z = config.spawn_x, config.spawn_y, config.spawn_z
    
    # Simple test offsets around the known spawn
    test_spawns = [
        (base_x, base_y, 0.0),
        (base_x + 1.0, base_y, 0.5),
        (base_x, base_y + 1.0, -0.5),
        (base_x - 1.0, base_y - 1.0, 1.57),
        (base_x + 2.0, base_y + 2.0, -1.57)
    ]

    env = IsaacDirectEnv(config=config, simulation_app=simulation_app)
    
    results = []
    
    try:
        for i, (x, y, yaw) in enumerate(test_spawns):
            print(f"\n[Test {i+1}/5] Spawning at X={x:.2f}, Y={y:.2f}, Yaw={yaw:.2f}...")
            
            # Manually override config for this reset
            env.config.spawn_x = x
            env.config.spawn_y = y
            env.config.spawn_yaw = yaw
            
            obs, info = env.reset()
            
            # Track stability over 50 steps
            stable_steps = 0
            img_means = []
            max_vel = 0.0
            has_nan = False
            
            for step in range(50):
                # Apply zero action (stationary test)
                action = np.zeros(3)
                obs, reward, terminated, truncated, info = env.step(action)
                
                # 1. Check Visuals (Exposure)
                img = obs["image"]
                img_mean = np.mean(img)
                img_means.append(img_mean)
                
                # 2. Check Physics (Explosions)
                vec = obs["vec"]
                speed = vec[3]
                max_vel = max(max_vel, speed)
                
                if np.any(np.isnan(vec)):
                    print(f"  [CRITICAL] NaN detected at step {step}!")
                    has_nan = True
                    break
                
                if speed > 50.0: # Abnormal velocity for stationary robot
                    print(f"  [CRITICAL] Velocity explosion detected! Speed: {speed:.2f} m/s")
                    break
                
                stable_steps += 1
            
            # Summary for this spawn point
            avg_exposure = np.mean(img_means) if img_means else 0
            # A "PASS" means it stayed relatively still and didn't explode
            status = "PASS" if (not has_nan and max_vel < 0.5) else "FAIL"
            
            print(f"  - Status: {status}")
            print(f"  - Avg Exposure (Mean): {avg_exposure:.2f}")
            print(f"  - Max Jitter Velocity: {max_vel:.4f} m/s")
            
            results.append({
                "spawn": (x, y),
                "status": status,
                "exposure": avg_exposure,
                "max_vel": max_vel
            })

    except Exception as e:
        print(f"\n[ERROR] Verification interrupted: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("\n--- Final Verification Summary ---")
        if not results:
            print("No tests completed.")
        for i, res in enumerate(results):
            print(f"Test {i+1}: {res['status']} | Exposure: {res['exposure']:.1f} | MaxVel: {res['max_vel']:.4f}")
        
        print("\nClosing Environment...")
        env.close()
        simulation_app.close()

if __name__ == "__main__":
    verify_stability()
