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
    yaw = yaw + 3.14159265
    yaw = torch.atan2(torch.sin(yaw), torch.cos(yaw))
    
    lat_err, raw_head_err, kappa = tm.compute_errors(local_pos, yaw)
    if "track_dir" not in env.extras:
        env.extras["track_dir"] = torch.where(torch.abs(raw_head_err) > 1.5708, -1.0, 1.0).to(env.device)
    is_reverse = env.extras["track_dir"] < 0
    head_err = torch.where(is_reverse, raw_head_err - 3.14159 * torch.sign(raw_head_err), raw_head_err)
    lat_err = torch.where(is_reverse, -lat_err, lat_err)
    
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
    # The car was flipped to drive in +X (v1.0-working), but root X points backward!
    vel = -asset.data.root_lin_vel_b[:, 0]
    # Allow 0 velocity if aligned or if it's the first few steps of the episode
    moving_forward = vel > 0.05
    
    # Check if we just started (less than 20 steps) to prevent spawn resets
    is_spawn = env.episode_length_buf < 20
    
    # Gate intent logic: Alignment OK if looking straight OR if we just spawned
    aligned_lat = torch.abs(torch.sin(head_err)) > 0.8 
    alignment_ok = (aligned_long | aligned_lat | is_spawn) & (moving_forward | is_spawn)
    
    gate_hit = gate_contact & (~alignment_ok)
    
    # Apply grace period to both boundaries and gates
    return (~is_spawn) & (masked_boundary_hit | gate_hit)

def height_termination(env: ManagerBasedRLEnv, threshold: float = -0.05, max_height: float = 0.50, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    """Terminates if the robot flips, falls below a threshold, or flies too high."""
    asset = env.scene[asset_cfg.name]
    z_pos = asset.data.root_pos_w[:, 2]
    # Add a grace period to prevent immediate failures on spawn settling
    is_spawn = env.episode_length_buf < 20
    return (~is_spawn) & ((z_pos < threshold) | (z_pos > max_height))

def stagnation_termination(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    """Terminates if the robot is crawling or stuck, measured by waypoint progress."""
    past_grace = env.episode_length_buf > 100
    
    cum_wp = env.extras.get("cumulative_wp_index", None)
    if cum_wp is None:
        return torch.zeros(env.num_envs, dtype=torch.bool, device=env.device)
        
    if "stagnation_term_snapshot" not in env.extras:
        env.extras["stagnation_term_snapshot"] = cum_wp.clone()
        env.extras["stagnation_term_timer"] = torch.zeros(env.num_envs, dtype=torch.long, device=env.device)
        
    timer = env.extras["stagnation_term_timer"]
    snapshot = env.extras["stagnation_term_snapshot"]
    
    timer += 1
    
    # Every 100 steps (2 seconds), it MUST have progressed at least 5 waypoints (~0.3m)
    check_now = timer >= 100
    progress = cum_wp - snapshot
    
    stagnated = (progress < 5.0) & check_now & past_grace
    
    # Reset snapshot for envs that were checked
    env.extras["stagnation_term_snapshot"] = torch.where(check_now, cum_wp, snapshot)
    env.extras["stagnation_term_timer"] = torch.where(check_now, torch.zeros_like(timer), timer)
    
    # Also reset on episode reset
    if hasattr(env, 'episode_length_buf'):
        just_reset = env.episode_length_buf <= 1
        env.extras["stagnation_term_snapshot"] = torch.where(just_reset, cum_wp, env.extras["stagnation_term_snapshot"])
        env.extras["stagnation_term_timer"] = torch.where(just_reset, torch.zeros_like(timer), env.extras["stagnation_term_timer"])
        
    return stagnated

def fov_visibility_termination(env: ManagerBasedRLEnv, horizontal_aperture: float, focal_length: float, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    """Terminates if the track centerline heading error exceeds half the camera FOV (driving blind)."""
    import math
    head_err = env.extras.get("head_err", torch.zeros(env.num_envs, device=env.device))
    # FOV = 2 * atan( h_aperture / (2 * f_length) )
    fov_rad = 2.0 * math.atan(horizontal_aperture / (2.0 * focal_length))
    # If heading error is greater than half the FOV, the track is out of sight
    return torch.abs(head_err) > (fov_rad / 2.0)

def ground_contact_termination(env: ManagerBasedRLEnv, sensor_cfg: SceneEntityCfg, threshold: float = 1.0) -> torch.Tensor:
    """Terminates if the robot (any part) physically touches the ground plane."""
    contact_sensor = env.scene[sensor_cfg.name]
    net_contact_forces = contact_sensor.data.net_forces_w_history
    # Shape: (num_envs, history_length, num_bodies, 3)
    # Compute max force across the XYZ dimension
    force_magnitudes = torch.norm(net_contact_forces, dim=-1)
    # Max across bodies
    max_body_force = torch.max(force_magnitudes, dim=-1)[0]
    # Max across history
    max_force = torch.max(max_body_force, dim=-1)[0]
    
    is_spawn = env.episode_length_buf < 20
    return (~is_spawn) & (max_force > threshold)
