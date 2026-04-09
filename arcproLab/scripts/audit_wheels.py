import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Audit wheel directions.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import torch
import numpy as np
from isaaclab.envs import ManagerBasedRLEnv
from arcproLab.arcpro_env_cfg import ARCProEnvCfg

def main():
    env_cfg = ARCProEnvCfg()
    env_cfg.scene.num_envs = 1
    env = ManagerBasedRLEnv(cfg=env_cfg)
    
    print("\n" + "="*60)
    print("WHEEL DIRECTION AUDIT")
    print("="*60)
    
    # Test each drive joint (Indices 2, 3, 4, 5)
    drive_indices = [2, 3, 4, 5]
    names = ["RL", "RR", "FL", "FR"]
    
    for idx, name in zip(drive_indices, names):
        print(f"\nTesting Wheel: {name} (Index {idx})")
        # Reset
        env.reset()
        # Apply -20 rad/s to just THIS wheel for 50 steps
        for _ in range(50):
            actions = torch.zeros((1, 6), device=env.device)
            actions[0, idx] = -20.0 
            env.step(actions)
        
        # Check actual velocity
        actual_v = env.scene["robot"].data.joint_vel[0, idx].item()
        lin_vel = env.scene["robot"].data.root_lin_vel_b[0, 0].item()
        print(f"  Commanded: -20.0 | Actual JV: {actual_v:.2f} | Body Move X: {lin_vel:.4f}")
        
        if lin_vel > 0.01:
            print(f"  RESULT: -Command moves car FORWARD (Correct)")
        elif lin_vel < -0.01:
            print(f"  RESULT: -Command moves car BACKWARD (Inverted!)")
        else:
            print(f"  RESULT: No significant movement.")

    print("="*60 + "\n")
    env.close()
    simulation_app.close()

if __name__ == "__main__":
    main()
