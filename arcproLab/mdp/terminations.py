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
    
    # Standard boundaries (Always reset if hitting the edge)
    # White (Edge) or Yellow (Center)
    boundary_hit = (dist_w < 0.05) | (dist_y < 0.05)
    
    # DEBUG: Loud diagnostic for Env 0
    if torch.rand(1).item() < 0.05: # Sample 5% of steps
        step_count = env.unwrapped.episode_length_buf[0].item()
        print(f"[DIAGNOSTIC] Env 0 | Step: {step_count} | DistW: {dist_w[0].item():.4f} | Hit: {boundary_hit[0].item()}")
    
    # Gates (Permeable if aligned)
    gate_contact = dist_g < 0.12
    
    # Calculate yaw from quaternion
    q = asset.data.root_quat_w
    yaw = torch.atan2(2.0 * (q[:, 0] * q[:, 3] + q[:, 1] * q[:, 2]), 1.0 - 2.0 * (q[:, 2]**2 + q[:, 3]**2))
    
    # Get heading error
    _, head_err, _ = tm.compute_errors(asset.data.root_pos_w[:, :2] - env.scene.env_origins[:, :2], yaw)
    
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
    return torch.zeros(env.num_envs, dtype=torch.bool, device=env.device)

def height_termination(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    """Terminates if the robot flips or falls."""
    asset = env.scene[asset_cfg.name]
    height = asset.data.root_pos_w[:, 2]
    term = (height < -0.1) | (height > 5.0)
    if term[0].item() and torch.rand(1).item() < 0.1:
        print(f"[DIAGNOSTIC] Env 0 Height Term! H: {height[0].item():.4f}")
    return term

def stagnation_termination(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    return torch.zeros(env.num_envs, dtype=torch.bool, device=env.device)
