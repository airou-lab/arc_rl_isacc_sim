# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import torch
from isaaclab.managers import SceneEntityCfg
from isaaclab.envs import ManagerBasedRLEnv

def white_line_contact(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"), threshold: float = 0.15) -> torch.Tensor:
    """
    Terminates if the robot hits a white line boundary.
    Mastery Logic: Relaxed margin (0.15m) to prevent physics-induced resets, 
    but still resets if leaving the road.
    """
    from mdp.track_manager import get_track_manager
    tm = get_track_manager(device=env.device)
    
    asset = env.scene[asset_cfg.name]
    env_origins = env.scene.env_origins
    local_pos = asset.data.root_pos_w - env_origins
    
    # FIX: 1-Step Delay. Compute lat_err and head_err here (during terminations) 
    # so that the Reward Manager has the current step's data, not the previous step's!
    q = asset.data.root_quat_w
    yaw = torch.atan2(2.0 * (q[:, 0] * q[:, 3] + q[:, 1] * q[:, 2]), 1.0 - 2.0 * (q[:, 2]**2 + q[:, 3]**2))
    lat_err, head_err, kappa = tm.compute_errors(local_pos, yaw)
    env.extras["lat_err"] = lat_err
    env.extras["head_err"] = head_err
    
    # Get boundary data
    dist_y, dist_w, dist_g = tm.compute_marker_distances(local_pos)
    
    # 1. Boundary hit (White or Yellow line dist < threshold)
    # Default: 0.15m (Provides buffer for 50Hz control loop)
    # Also add a fallback: if lateral error > 3.5m, it is definitely off the road.
    boundary_hit = (dist_w < threshold) | (dist_y < threshold) | (torch.abs(lat_err) > 3.5)

    
    # 2. Gate permeability check (Phase 11-17)
    # Mask the boundary reset if we are inside a gate zone (dist_g < 0.50m)
    # This allows the robot to cross white lines that belong to gates
    in_gate_zone = dist_g < 0.50
    masked_boundary_hit = boundary_hit & (~in_gate_zone)
    
    # 3. Gate contact logic: Still resets if entering gate with bad alignment
    gate_contact = dist_g < 0.12
    
    # Check alignment (Heading error < 45 degrees)
    head_err = env.extras.get("head_err", torch.zeros(env.num_envs, device=env.device))
    aligned_long = torch.abs(head_err) < 0.8  # ~45 deg
    
    # Also check if moving forward (Grace period for spawn)
    vel = asset.data.root_lin_vel_b[:, 0]
    # Allow 0 velocity if aligned or if it's the first few steps of the episode
    moving_forward = vel > 0.05
    
    # Check if we just started (less than 5 steps) to prevent spawn resets
    is_spawn = env.episode_length_buf < 5
    
    # Gate intent logic: Alignment OK if looking straight OR if we just spawned
    aligned_lat = torch.abs(torch.sin(head_err)) > 0.8 
    alignment_ok = (aligned_long | aligned_lat | is_spawn) & (moving_forward | is_spawn)
    
    gate_hit = gate_contact & (~alignment_ok)
    
    # Apply grace period to both boundaries and gates
    return (~is_spawn) & (masked_boundary_hit | gate_hit)

def height_termination(env: ManagerBasedRLEnv, threshold: float = 0.01, max_height: float = 0.50, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    """Terminates if the robot flips, falls below a threshold, or flies too high."""
    asset = env.scene[asset_cfg.name]
    z_pos = asset.data.root_pos_w[:, 2]
    return (z_pos < threshold) | (z_pos > max_height)

def stagnation_termination(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    """Terminates if the robot is crawling or stuck (speed < 0.1 m/s)."""
    asset = env.scene[asset_cfg.name]
    # Use absolute speed magnitude in the XY plane to avoid axis polarity issues
    vel = torch.norm(asset.data.root_lin_vel_b[:, :2], dim=1)
    # Reset if speed is less than 0.1 m/s for more than 1000 steps (~20.0s)
    # Relaxed to allow the agent to explore longer paths without being killed by the timer.
    return (vel < 0.1) & (env.episode_length_buf > 1000)

def fov_visibility_termination(env: ManagerBasedRLEnv, horizontal_aperture: float, focal_length: float, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    """Terminates if the track centerline heading error exceeds half the camera FOV (driving blind)."""
    import math
    head_err = env.extras.get("head_err", torch.zeros(env.num_envs, device=env.device))
    # FOV = 2 * atan( h_aperture / (2 * f_length) )
    fov_rad = 2.0 * math.atan(horizontal_aperture / (2.0 * focal_length))
    # If heading error is greater than half the FOV, the track is out of sight
    return torch.abs(head_err) > (fov_rad / 2.0)
