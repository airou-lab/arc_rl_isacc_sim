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
parser.add_argument("--max_steps", type=int, default=1000, help="Maximum number of simulation steps.")
# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
# parse the arguments
args_cli = parser.parse_args()
args_cli.enable_cameras = True # Hardcode for TiledCamera support

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
    env_cfg.enable_cameras = True # FORCE ENABLE for policy
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
    max_steps = args_cli.max_steps
    actions = torch.zeros((env.num_envs, 6), device=env.device)
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
                # Action shape is 6 (2 steering + 4 throttle)
                actions = torch.zeros((env.num_envs, 6), device=env.device)

                for i in range(env.num_envs):
                    img = images[i]
                    prediction = policy.predict(img)
                    steering, throttle = policy.get_action(prediction)

                    # Target velocity (rad/s) for 8x car
                    # +40.0 is forward (Audit showed -20.0 moves car BACKWARD)
                    target_rad_s = 40.0 

                    # Corrected Joint Mapping (Audit Result):
                    # Index 0, 1: Steering (L, R)
                    # Index 2, 3: Rear Drive (RL, RR)
                    # Index 4, 5: Front Drive (FL, FR)

                    # Steering (Position Control)
                    actions[i, 0] = steering
                    actions[i, 1] = steering

                    # Throttle (Velocity Control) - AWD
                    actions[i, 2] = target_rad_s # RL
                    actions[i, 3] = target_rad_s # RR
                    actions[i, 4] = target_rad_s # FL
                    actions[i, 5] = target_rad_s # FR
            else:
                # FALLBACK: Constant forward if camera failed
                actions = torch.zeros((env.num_envs, 6), device=env.device)
                actions[:, 4:6] = 40.0 # Drive FL and FR forward

            # step environment
            obs, rewards, terminated, truncated, info = env.step(actions)
            
            # Note: Isaac Lab automatically resets terminated envs during step()
            if terminated.any() or truncated.any():
                if count % 20 == 0:
                    print(f"  [!] Reset triggered (Terminated: {terminated.any()}, Truncated: {truncated.any()})")

            # Telemetry for env 0 (Outside of image block for visibility)
            if count % 20 == 0:
                 pos = env.scene["robot"].data.root_pos_w
                 vel = env.scene["robot"].data.root_lin_vel_w
                 v_norm = torch.linalg.norm(vel[0]).item()

                 lat_err_val = "N/A"
                 obs_lat_err = "N/A"
                 if tm is not None:
                     q = env.scene["robot"].data.root_quat_w
                     yaw = torch.atan2(2.0 * (q[:, 0] * q[:, 3] + q[:, 1] * q[:, 2]), 1.0 - 2.0 * (q[:, 2]**2 + q[:, 3]**2))
                     lat_err, head_err = tm.compute_errors(pos, yaw)
                     lat_err_val = f"{lat_err[0].item():.3f}m"

                     # Read directly from observation manager for parity check
                     obs_dict = env.observation_manager.compute()["policy"]
                     obs_lat_err = f"{obs_dict[0, 8].item():.4f}"

                 # Audit joint velocities
                 jv = env.scene["robot"].data.joint_vel[0, :]
                 jv_drive = jv.cpu().numpy().tolist()

                 # Steering value (from last action)
                 steer_val = actions[0, 0].item()

                 print(f"Step {count:4d} | Pos: ({pos[0,0]:.2f}, {pos[0,1]:.2f}, {pos[0,2]:.3f}) | Vel: {v_norm:.2f}m/s")
                 print(f"          | LatErr (m): {lat_err_val} | LatErr (norm): {obs_lat_err} | Limit: 0.075")
                 print(f"          | Steer: {steer_val:.3f} | JV: {jv_drive}")
                 if telemetry is not None:
                     telemetry.update(count, v_norm, steer_val, lat_err_val, jv_drive)
            
            if images is None and count % 20 == 0:
                print(f"  [!] Note: Driving Blind (No visual obs)")
        
        count += 1

    # close environment
    env.close()

if __name__ == "__main__":
    main()
