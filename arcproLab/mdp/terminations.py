# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import torch
from isaaclab.managers import SceneEntityCfg
from isaaclab.envs import ManagerBasedRLEnv

def white_line_contact(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    """Terminates if the robot hits a white line boundary."""
    from mdp.track_manager import get_track_manager
    tm = get_track_manager(device=env.device)
    
    asset = env.scene[asset_cfg.name]
    env_origins = env.scene.env_origins
    local_pos = asset.data.root_pos_w - env_origins
    
    # Get boundary data
    dist_y, dist_w, dist_g = tm.compute_marker_distances(local_pos)
    
    # 1. Boundary hit (White or Yellow line dist < threshold)
    # 0.13m allows ~3cm of wiggle room for the 0.28m wide robot in a 0.60m lane
    boundary_hit = (dist_w < 0.13) | (dist_y < 0.13)
    
    # 2. Gate permeability check (Phase 11-17)
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
    return (~is_spawn) & (boundary_hit | gate_hit)

def height_termination(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    """Terminates if the robot flips or falls."""
    asset = env.scene[asset_cfg.name]
    return asset.data.root_pos_w[:, 2] < 0.05

def stagnation_termination(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    """Terminates if the robot stays still for too long (stuck against wall)."""
    return torch.zeros(env.num_envs, dtype=torch.bool, device=env.device)

def fov_visibility_termination(env: ManagerBasedRLEnv, horizontal_aperture: float, focal_length: float, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    return torch.zeros(env.num_envs, dtype=torch.bool, device=env.device)
