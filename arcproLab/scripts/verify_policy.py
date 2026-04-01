# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""
Script to verify the SB3 policy in the Isaac Lab environment.
"""

import argparse
from isaaclab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(description="Verify the SB3 policy in the Isaac Lab environment.")
parser.add_argument("--num_envs", type=int, default=1, help="Number of environments to spawn.")
# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
# parse the arguments
args_cli = parser.parse_args()

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest of the imports."""
import torch
import os
import sys
import numpy as np

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
from mdp.policy_wrapper import PolicyWrapper
from mdp.track_manager import get_track_manager

# Conditional import for UI
if not args_cli.headless:
    from mdp.visual_analytics import TelemetryWindow
else:
    TelemetryWindow = None

def main():
    # setup configuration
    env_cfg = ARCProEnvCfg()
    env_cfg.scene.num_envs = args_cli.num_envs
    env_cfg.enable_cameras = args_cli.enable_cameras
    env_cfg.__post_init__() 
    
    # setup environment
    print("Initializing ManagerBasedRLEnv...")
    env = ManagerBasedRLEnv(cfg=env_cfg)
    print("ManagerBasedRLEnv initialized.")
    
    # setup policy
    print("Loading SB3 policy...")
    model_path = os.path.join(os.path.dirname(__file__), "..", "models", "road_following_model.pth")
    policy = PolicyWrapper(model_path)
    print("Policy loaded.")
    
    # setup track manager for error tracking
    try:
        tm = get_track_manager(device=env.device)
        print("TrackManager initialized.")
    except Exception as e:
        tm = None
        print(f"TrackManager NOT initialized: {e}")
    
    # reset environment
    print("Resetting environment...")
    obs, _ = env.reset()
    print("Environment reset complete.")
    
    # Alignment Check
    if tm is not None:
        p0 = env.scene["robot"].data.root_pos_w[0]
        q0 = env.scene["robot"].data.root_quat_w[0]
        y0 = torch.atan2(2.0 * (q0[0] * q0[3] + q0[1] * q0[2]), 1.0 - 2.0 * (q0[2]**2 + q0[3]**2))
        l_err, h_err = tm.compute_errors(p0.unsqueeze(0), y0.unsqueeze(0))
        print(f"[Alignment Check] Robot Spawn: {p0.cpu().numpy()}")
        print(f"[Alignment Check] Nearest WP Error: Lat={l_err.item():.3f}m, Hdg={h_err.item():.3f}rad")

    # Setup Telemetry Window
    telemetry = None
    if TelemetryWindow is not None:
        print("Creating Telemetry Window...")
        telemetry = TelemetryWindow()
    
    # Find joint indices for telemetry audit
    drive_names = ["Joint_Drive_FL", "Joint_Drive_FR", "Joint_Drive_RL", "Joint_Drive_RR"]
    drive_indices, _ = env.scene["robot"].find_joints(drive_names)

    # simulation loop
    count = 0
    max_steps = 20000 
    print(f"Starting simulation loop for {max_steps} steps...")
    while simulation_app.is_running() and count < max_steps:
        # Mandatory GUI update for real-time visibility
        if not args_cli.headless:
            simulation_app.update()

        with torch.no_grad():
            # Get camera observations
            images = None
            if "visual" in obs:
                v_data = obs["visual"]
                if isinstance(v_data, dict):
                    images = v_data.get("tiled_camera")
                else:
                    images = v_data 
                
            if images is not None:
                # Action shape is 4 (2 steering + 2 throttle) as per ActionCfg
                actions = torch.zeros((env.num_envs, 4), device=env.device)
                
                for i in range(env.num_envs):
                    img = images[i]
                    prediction = policy.predict(img)
                    steering, throttle = policy.get_action(prediction)
                    
                    # Target velocity (rad/s)
                    target_rad_s = 40.0 # ~2.0 m/s
                    
                    # Steering (Indices 0, 1) -> Joint_Steer_L, Joint_Steer_R
                    actions[i, 0] = steering
                    actions[i, 1] = steering
                    
                    # Throttle (Indices 2, 3) -> Joint_Drive_FL, FR
                    actions[i, 2] = -target_rad_s # FL
                    actions[i, 3] = -target_rad_s # FR
                    
                    # Telemetry for env 0 (12-float vector from policy group)
                    if i == 0:
                         tel = obs["policy"][0] # (12,)
                         
                         pos = env.scene["robot"].data.root_pos_w
                         vel = env.scene["robot"].data.root_lin_vel_w
                         v_norm = torch.linalg.norm(vel[0]).item()
                         
                         # Audit joint velocities
                         jv = env.scene["robot"].data.joint_vel[0, drive_indices]
                         jv_drive = jv.cpu().numpy().tolist()
                         
                         if count % 20 == 0:
                             print(f"Step {count:4d} | Pos: ({pos[0,0]:.2f}, {pos[0,1]:.2f}, {pos[0,2]:.3f}) | Vel: {tel[3]:.2f}m/s | LatErr: {tel[8]:.3f}m | HdgErr: {tel[9]:.3f}rad")
                             print(f"          | Steer: {steering:.3f} | Kappa: {tel[10]:.4f} | Dist: {tel[11]:.2f}m")
                         
                         if telemetry is not None:
                             # Update telemetry window with 12-float values
                             # Index mapping: [3] Speed, [5] Steer, [8] LatErr
                             telemetry.update(count, tel[3].item(), tel[5].item(), f"{tel[8].item():.3f}m", jv_drive)
                
                # step environment
                obs, reward, terminated, truncated, _ = env.step(actions)
                
                if count % 20 == 0:
                    print(f"          | Reward: {reward[0].item():.3f}")
            else:
                # Fallback: Just drive forward if no camera
                # Index 2, 3 are drive joints in the 4-dim action space
                actions = torch.zeros((env.num_envs, 4), device=env.device)
                actions[:, 2:] = -40.0
                obs, _, _, _, _ = env.step(actions)
                
                if count % 20 == 0:
                    print(f"Step {count:4d} | Driving Blind (No visual obs)")
        
        count += 1

    # close environment
    env.close()

if __name__ == "__main__":
    main()
