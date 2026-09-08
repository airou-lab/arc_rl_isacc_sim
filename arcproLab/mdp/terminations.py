# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import torch
from isaaclab.managers import SceneEntityCfg
from isaaclab.envs import ManagerBasedRLEnv

def white_line_contact(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"), threshold: float = 0.05) -> torch.Tensor:
    """
    Terminates if the robot hits a white or yellow line boundary.
    Configured to 0.05m (5cm) to allow tight inner-curb apex clipping (8.3cm clearance)
    while still immediately resetting if the vehicle departs the drivable road surface.
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
    
    # 1. Boundary hit (Apex-safe thresholds for 1.0x scale):
    # Outer white line: threshold 0.04m from chassis center.
    # Inner yellow curb: threshold 0.02m from chassis center (allows apex clipping at WP 986 where centerline is 6.8cm).
    # Shoulder departure: detects vehicle crossing outside white line (lat_err < -0.25m and farther from yellow than road width).
    # Lane departure fallback: lateral error > 0.60m from track centerline.
    threshold_w = 0.04
    threshold_y = 0.02
    off_road_shoulder = (lat_err < -0.25) & (dist_y > 0.38)
    boundary_hit = (dist_w < threshold_w) | (dist_y < threshold_y) | off_road_shoulder | (torch.abs(lat_err) > 0.60)

    # 2. Gate permeability check:
    # Mask boundary resets inside intersection gate zones (dist_g < 0.50m) where road markings cross path
    in_gate_zone = dist_g < 0.50
    masked_boundary_hit = boundary_hit & (~in_gate_zone)

    # Check if we just started (less than 20 steps) to prevent spawn resets
    is_spawn = env.episode_length_buf < 20

    # Apply grace period
    return (~is_spawn) & masked_boundary_hit

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
    # Stagnation is suppressed when dwelling at a stop bar / red signal (go_signal == 0)
    check_now = timer >= 100
    progress = cum_wp - snapshot
    is_go = env.extras.get("go_signal", torch.ones(env.num_envs, device=env.device)) > 0.5
    
    stagnated = (progress < 5.0) & check_now & past_grace & is_go
    
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
