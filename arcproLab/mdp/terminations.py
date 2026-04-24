# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import torch
from isaaclab.managers import SceneEntityCfg
from isaaclab.envs import ManagerBasedRLEnv

def white_line_contact(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    """
    Terminates if the robot center gets too close to ANY road marker.
    Ditches centerline math for robust direct proximity.
    Supports permeable Gate markers (Stop Lines).
    """
    from mdp.track_manager import get_track_manager
    tm = get_track_manager(device=env.device)
    asset = env.scene[asset_cfg.name]
    
    # Get direct distance to Yellow (Center), White (Edge), and Gate (Stop) markers
    dist_y, dist_w, dist_g = tm.compute_marker_distances(asset.data.root_pos_w - env.scene.env_origins)
    
    # Standard boundaries (Always reset)
    boundary_hit = (dist_y < 0.12) | (dist_w < 0.12)
    
    # Gates (Permeable if aligned)
    gate_contact = dist_g < 0.12
    
    # Calculate yaw from quaternion
    q = asset.data.root_quat_w
    yaw = torch.atan2(2.0 * (q[:, 0] * q[:, 3] + q[:, 1] * q[:, 2]), 1.0 - 2.0 * (q[:, 2]**2 + q[:, 3]**2))
    
    # Get heading error
    _, head_err = tm.compute_errors(asset.data.root_pos_w[:, :2] - env.scene.env_origins[:, :2], yaw)
    
    # Speed/Intent check (Task 11-16)
    # Only allow gate permeability if moving forward > 0.1m/s
    speed = asset.data.root_lin_vel_b[:, 0]
    moving_forward = speed > 0.1
    
    # Only reset on Gate if alignment is poor (sliding into it) OR if we are stationary
    # Alignment: cos(head_err) > 0.7 means robot is facing roughly forward/backward along the path
    # We also check sin(head_err) to allow 90 degree crossing (perpendicular gates in intersections)
    cos_err = torch.cos(head_err)
    sin_err = torch.sin(head_err)
    
    # facing 0 or 180 (aligned with lane)
    aligned_long = torch.abs(cos_err) > 0.707 
    # facing 90 or 270 (crossing perpendicular stop lines)
    aligned_lat = torch.abs(sin_err) > 0.707
    
    alignment_ok = (aligned_long | aligned_lat) & moving_forward
    
    # A gate hit is ONLY a termination if alignment/intent is NOT okay
    gate_hit = gate_contact & (~alignment_ok)
    
    return boundary_hit | gate_hit

def fov_visibility_termination(env: ManagerBasedRLEnv, horizontal_aperture: float, focal_length: float, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    """Terminates if the robot drives in a direction outside its camera's FOV."""
    import math
    half_fov = math.atan(horizontal_aperture / (2.0 * focal_length))
    asset = env.scene[asset_cfg.name]
    vel_b = asset.data.root_lin_vel_b
    driving_angle = torch.atan2(vel_b[:, 1], vel_b[:, 0])
    speed = torch.norm(vel_b[:, :2], dim=-1)
    settled = env.episode_length_buf > 20
    out_of_view = settled & (speed > 0.5) & (torch.abs(driving_angle) > half_fov)
    return out_of_view

def height_termination(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    """Terminates if the robot flips or falls."""
    asset = env.scene[asset_cfg.name]
    height = asset.data.root_pos_w[:, 2]
    return (height < 0.02) | (height > 5.0)

def stagnation_termination(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    """Terminates if the robot hasn't made forward progress."""
    if "last_dist" not in env.extras:
        env.extras["last_dist"] = torch.zeros(env.num_envs, device=env.device)
        env.extras["stagnant_steps"] = torch.zeros(env.num_envs, device=env.device)
    current_dist = env.extras.get("distance", torch.zeros(env.num_envs, device=env.device))
    progress = (current_dist - env.extras["last_dist"]) > 0.01
    env.extras["stagnant_steps"] = torch.where(progress, torch.zeros_like(env.extras["stagnant_steps"]), env.extras["stagnant_steps"] + 1)
    env.extras["last_dist"] = current_dist.clone()
    return env.extras["stagnant_steps"] > 500
