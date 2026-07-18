# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import argparse
from isaaclab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(description="Verify the SB3 policy in the Isaac Lab environment.")
parser.add_argument("--num_envs", type=int, default=1, help="Number of environments to spawn.")
parser.add_argument("--checkpoint", type=str, required=True, help="Path to the model checkpoint.")
parser.add_argument("--waypoint_index", type=int, default=None, help="Start at a specific waypoint index (0-999).")
# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
# parse the arguments
args_cli = parser.parse_args()
args_cli.enable_cameras = True 

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import torch
import os
import sys
import numpy as np
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import VecNormalize

# Add both root and arcproLab to sys.path
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, "..", ".."))
ARCPRO_LAB_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, ".."))

if ROOT_DIR not in sys.path:
    sys.path.append(ROOT_DIR)
if ARCPRO_LAB_DIR not in sys.path:
    sys.path.append(ARCPRO_LAB_DIR)

from isaaclab.envs import ManagerBasedRLEnv
from arcpro_env_cfg import ARCProEnvCfg
from isaaclab_rl.sb3 import Sb3VecEnvWrapper

def main():
    # 1. Setup configuration
    env_cfg = ARCProEnvCfg()
    env_cfg.scene.num_envs = args_cli.num_envs
    env_cfg.enable_cameras = True 
    env_cfg.__post_init__() 
    
    # 2. Create environment
    env = ManagerBasedRLEnv(cfg=env_cfg)
    # Wrap for SB3
    env = Sb3VecEnvWrapper(env)
    
    # 3. Load Normalization
    checkpoint_dir = os.path.dirname(args_cli.checkpoint)
    stats_path = os.path.join(checkpoint_dir, "vec_normalize.pkl")
    
    if os.path.exists(stats_path):
        print(f"Loading normalization stats from: {stats_path}")
        env = VecNormalize.load(stats_path, env)
        env.training = False
        env.norm_reward = False
    
    # 4. Load model
    print(f"Loading checkpoint: {args_cli.checkpoint}")
    model = PPO.load(args_cli.checkpoint, env=env)
    
    # 5. Reset environment
    obs = env.reset()
    
    # 6. Optional: Manual Teleport to Waypoint
    if args_cli.waypoint_index is not None:
        wp_path = os.path.join(ARCPRO_LAB_DIR, "mdp", "track_centerline.npy")
        if os.path.exists(wp_path):
            wps = np.load(wp_path)
            if args_cli.waypoint_index < len(wps):
                target_wp = wps[args_cli.waypoint_index]
                print(f"Teleporting to Waypoint {args_cli.waypoint_index}: {target_wp}")
                
                import math
                yaw = target_wp[2]
                qw = math.cos(yaw/2.0)
                qz = math.sin(yaw/2.0)
                
                raw_env = env.unwrapped
                asset = raw_env.scene["robot"]
                
                pose = torch.tensor([[target_wp[0], target_wp[1], 1.5, qw, 0.0, 0.0, qz]], device=raw_env.device)
                asset.write_root_pose_to_sim(pose)
                asset.write_root_velocity_to_sim(torch.zeros((1, 6), device=raw_env.device))
                print("Teleport complete.")

    # 7. Simulation loop
    print("Starting visual verification (STRICT TERMINATIONS ACTIVE)...")
    step_count = 0
    while simulation_app.is_running():
        with torch.no_grad():
            actions, _ = model.predict(obs, deterministic=True)
            obs, rewards, dones, info = env.step(actions)
            
            if step_count % 10 == 0:
                raw_env = env.unwrapped
                raw_lat_err = raw_env.extras.get("raw_lat_err", torch.tensor([0.0]))[0].item()
                raw_speed = raw_env.extras.get("raw_speed", torch.tensor([0.0]))[0].item()
                steer = actions[0][0].item()
                throttle = actions[0][1].item()
                print(f"Step {step_count:4} | LatErr: {raw_lat_err:6.3f}m | Speed: {raw_speed:5.2f}m/s | Act: [S:{steer:5.2f}, T:{throttle:5.2f}]")
            
        simulation_app.update()
        step_count += 1

    env.close()
    simulation_app.close()

if __name__ == "__main__":
    main()
