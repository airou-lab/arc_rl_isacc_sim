import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Deep verify drive direction.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import os
import sys
import torch
import numpy as np
from isaaclab.envs import ManagerBasedRLEnv
sys.path.append(os.path.join(os.path.dirname(__file__), ".."))
from arcpro_env_cfg import ARCProEnvCfg

def main():
    env_cfg = ARCProEnvCfg()
    env_cfg.scene.num_envs = 1
    env = ManagerBasedRLEnv(cfg=env_cfg)
    
    print("\n" + "="*60)
    print("DEEP DIRECTION VERIFICATION")
    print("="*60)
    
    env.reset()
    
    # 1. Get initial distance to next waypoint
    from mdp.track_manager import get_track_manager
    tm = get_track_manager(device=env.device)
    
    robot = env.scene["robot"]
    pos = robot.data.root_pos_w[0, :3] - env.scene.env_origins[0]
    
    # Find closest waypoint index
    wp = torch.tensor(np.load("arcproLab/mdp/track_centerline_1x.npy"), device=env.device)
    dists = torch.norm(wp[:, :2] - pos[:2], dim=1)
    start_idx = torch.argmin(dists).item()
    print(f"Starting at Waypoint Index: {start_idx}")
    
    # Apply +1.0 throttle
    action = torch.zeros((1, 3), device=env.device)
    action[0, 1] = 1.0 # Throttle
    
    print("Applying +1.0 throttle for 100 steps...")
    for i in range(100):
        env.step(action)
        
    # 2. Check final position
    pos_final = robot.data.root_pos_w[0, :3] - env.scene.env_origins[0]
    dists_final = torch.norm(wp[:, :2] - pos_final[:2], dim=1)
    final_idx = torch.argmin(dists_final).item()
    
    print(f"Final Waypoint Index: {final_idx}")
    
    # The track is indexed 0 to 1182. Forward is increasing index.
    if final_idx > start_idx:
        print("RESULT: Car is moving FORWARD along the track.")
    elif final_idx < start_idx:
        print("RESULT: Car is moving BACKWARD along the track!")
    else:
        print("RESULT: Car didn't move enough to change waypoint index.")
        
    lin_vel = robot.data.root_lin_vel_b[0, 0].item()
    print(f"Body Vel X: {lin_vel:.4f}")
    
    if lin_vel > 0 and final_idx > start_idx:
        print("POLARITY: Positive Body X = Forward Track. Correct.")
    elif lin_vel < 0 and final_idx > start_idx:
        print("POLARITY: Negative Body X = Forward Track. Body X is BACKWARDS.")
    elif lin_vel > 0 and final_idx < start_idx:
        print("POLARITY: Positive Body X = Backward Track. Action Inverted.")
        
    print("="*60 + "\n")
    env.close()
    simulation_app.close()

if __name__ == "__main__":
    main()
