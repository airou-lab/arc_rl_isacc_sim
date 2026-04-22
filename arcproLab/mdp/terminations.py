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
    """
    from mdp.track_manager import get_track_manager
    tm = get_track_manager(device=env.device)
    asset = env.scene[asset_cfg.name]
    
    # Get direct math error in meters
    dist_y, dist_w = tm.compute_marker_distances(asset.data.root_pos_w - env.scene.env_origins)
    
    # Reset if any part of the robot hits the line markers
    # 0.12m threshold allows the robot to exist in the ~0.44m wide lane
    inner_hit = dist_y < 0.12
    outer_hit = dist_w < 0.12
    marker_hit = inner_hit | outer_hit

    return marker_hit

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
    return (height < 0.02) | (height > 0.5)

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
