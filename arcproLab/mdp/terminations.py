# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import torch
from isaaclab.managers import SceneEntityCfg
from isaaclab.envs import ManagerBasedRLEnv

def white_line_contact(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    """
    Terminates if the robot touches the yellow or white road boundaries.
    """
    from mdp.track_manager import get_track_manager
    tm = get_track_manager(device=env.device)
    asset = env.scene[asset_cfg.name]
    
    # Get environment-relative position
    env_origins = env.scene.env_origins
    local_pos = asset.data.root_pos_w - env_origins
    
    # extract yaw from quaternion
    q = asset.data.root_quat_w
    yaw = torch.atan2(2.0 * (q[:, 0] * q[:, 3] + q[:, 1] * q[:, 2]), 1.0 - 2.0 * (q[:, 2]**2 + q[:, 3]**2))
    
    lat_err, _ = tm.compute_errors(local_pos, yaw)
    
    # Since TrackManager waypoints are exactly on the Yellow Line (lat_err = 0.0)
    # The robot is in the right lane (negative lat_err).
    
    # Yellow Line Hit: lat_err > 0.5m (Loosened to allow for exploration)
    inner_hit = lat_err > 0.5

    # White Line Hit: lat_err < -1.2m (Safe road boundary)
    outer_hit = lat_err < -1.2

    marker_hit = inner_hit | outer_hit

    # 0 grace period for physics stability (as requested)
    settled = torch.ones(env.num_envs, device=env.device, dtype=torch.bool)

    # Debug logging (env 0)
    if marker_hit[0].item() and (env.num_envs == 1 or env.scene.env_origins.shape[0] > 0):
        reason = "Yellow Line Hit" if inner_hit[0].item() else "White Line Hit"
        print(f"[TERMINATION] {reason}! LatErr: {lat_err[0].item():.3f}m | Limits: -1.2 to 0.5")


    return settled & marker_hit

def fov_visibility_termination(env: ManagerBasedRLEnv, horizontal_aperture: float, focal_length: float, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    """
    Terminates if the robot drives in a direction outside its camera's FOV.
    """
    import math
    # Calculate half FOV in radians
    half_fov = math.atan(horizontal_aperture / (2.0 * focal_length))
    
    asset = env.scene[asset_cfg.name]
    # Linear velocity in chassis frame (base frame)
    vel_b = asset.data.root_lin_vel_b
    
    # Driving Angle in XY plane (local to chassis)
    driving_angle = torch.atan2(vel_b[:, 1], vel_b[:, 0])
    
    # Speed (magnitude) to avoid noise at rest
    speed = torch.norm(vel_b[:, :2], dim=-1)
    
    # Settling buffer: Only check FOV after robot has been alive for 20 steps
    settled = env.episode_length_buf > 20

    # Terminate if:
    # 1. Moving faster than 0.5 m/s (ignore spawn jitters)
    # 2. Driving angle is outside the camera's horizontal FOV
    # 3. Robot has settled after spawn
    out_of_view = settled & (speed > 0.5) & (torch.abs(driving_angle) > half_fov)
    
    if out_of_view.any() and (env.num_envs == 1 or env.scene.env_origins.shape[0] > 0):
        if out_of_view[0].item():
            print(f"[TERMINATION] Driving Blind! Speed: {speed[0].item():.2f}m/s | Angle: {math.degrees(driving_angle[0].item()):.1f} deg | FOV: {math.degrees(half_fov*2):.1f} deg")
            
    return out_of_view

def height_termination(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    """Terminates if the robot flips or falls."""
    asset = env.scene[asset_cfg.name]
    height = asset.data.root_pos_w[:, 2]
    # 0 grace period
    # Adjusted height for 1x car (Min 0.02m to avoid ground clipping, Max 0.5m for airtime)
    too_low = height < 0.02
    too_high = height > 0.5
    terminate = too_low | too_high
    
    if terminate[0].item() and (env.num_envs == 1 or env.scene.env_origins.shape[0] > 0):
        reason = "Too Low" if too_low[0].item() else "Too High"
        print(f"[TERMINATION] Height! {reason} (Z={height[0].item():.3f}m)")
        
    return terminate

def stagnation_termination(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    """Terminates if the robot hasn't made forward progress in 100 steps."""
    # 0 grace period for physics stability (as requested)
    settled = torch.ones(env.num_envs, device=env.device, dtype=torch.bool)
    
    # Initialize extras if not present
    if "last_dist" not in env.extras:
        env.extras["last_dist"] = torch.zeros(env.num_envs, device=env.device)
        env.extras["stagnant_steps"] = torch.zeros(env.num_envs, device=env.device)

    current_dist = env.extras.get("distance", torch.zeros(env.num_envs, device=env.device))
    
    # Check progress only if settled
    progress = (current_dist - env.extras["last_dist"]) > 0.01
    env.extras["stagnant_steps"] = torch.where(~settled | progress, 
                                               torch.zeros_like(env.extras["stagnant_steps"]), 
                                               env.extras["stagnant_steps"] + 1)
    
    # Update last distance for next check
    env.extras["last_dist"] = current_dist.clone()
    
    # Terminate if stuck for more than 500 steps AFTER settling
    stuck = env.extras["stagnant_steps"] > 500
    
    if stuck[0].item():
        print(f"[TERMINATION] Robot Stagnant! No progress for 500 steps.")
        
    return stuck
