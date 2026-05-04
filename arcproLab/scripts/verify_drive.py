import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Verify drive direction and wheel spin.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import os
import sys
import torch
from isaaclab.envs import ManagerBasedRLEnv
# Add arcproLab to path
sys.path.append(os.path.join(os.path.dirname(__file__), ".."))
from arcpro_env_cfg import ARCProEnvCfg

def main():
    env_cfg = ARCProEnvCfg()
    env_cfg.scene.num_envs = 1
    env = ManagerBasedRLEnv(cfg=env_cfg)
    
    print("\n" + "="*60)
    print("DRIVE DIRECTION & SPIN VERIFICATION")
    print("="*60)
    
    obs, _ = env.reset()
    
    # Apply +1.0 throttle (40 rad/s via scale) for 100 steps
    # Action space is [steer, throttle]
    # steering is index 0 (2 joints), throttle is index 1 (4 joints)
    # ActionManager flattens them. 
    # steering: joints 0,1. throttle: joints 2,3,4,5.
    
    action = torch.zeros((1, 3), device=env.device)
    action[0, 1] = 1.0 # Max throttle
    
    print("Applying +1.0 throttle...")
    
    for i in range(1000):
        obs, reward, terminated, truncated, info = env.step(action)
        
        if i % 50 == 0:
            lin_vel = env.scene["robot"].data.root_lin_vel_b[0, 0].item()
            print(f"Step {i:4d} | Body Vel X: {lin_vel:7.4f}")

    final_vel = env.scene["robot"].data.root_lin_vel_b[0, 0].item()
    print("-"*60)
    if final_vel > 0.1:
        print(f"RESULT: POSITIVE action moves car FORWARD. (Velocity: {final_vel:.2f})")
    elif final_vel < -0.1:
        print(f"RESULT: POSITIVE action moves car BACKWARD! (Velocity: {final_vel:.2f})")
    else:
        print(f"RESULT: Car is STUCK or spinning in place. (Velocity: {final_vel:.2f})")
    
    print("="*60 + "\n")
    env.close()
    simulation_app.close()

if __name__ == "__main__":
    main()
