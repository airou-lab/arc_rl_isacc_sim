# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import torch
from isaaclab.managers import SceneEntityCfg
from isaaclab.envs import ManagerBasedRLEnv

def hybrid_racer_reward(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    """
    Hybrid Racer Gaussian Reward Strategy.
    - Lane Reward: 2.0 * exp(-(lat_err^2) / 0.25)
    - Speed Reward: speed * 2.0
    - Collision Penalty: -50.0 if hit
    """
    asset = env.scene[asset_cfg.name]
    
    # Get telemetry for lateral error
    # Note: We compute it directly to avoid dependency on the observation manager's update cycle
    from mdp.track_manager import get_track_manager
    tm = get_track_manager(device=env.device)
    
    # 1. Lane Reward (Gaussian)
    lat_err, _ = tm.compute_errors(asset.data.root_pos_w, torch.zeros(env.num_envs, device=env.device))
    lane_reward = 2.0 * torch.exp(-(lat_err**2) / 0.25)
    
    # 2. Speed Reward
    speed = asset.data.root_lin_vel_b[:, 0]
    speed_reward = speed * 2.0
    
    # 3. Collision Penalty
    # Check for any contact with non-track objects if available, or just use termination status
    collision_penalty = torch.zeros(env.num_envs, device=env.device)
    if hasattr(env, "termination_manager"):
        # If any termination term starting with 'collision' or 'off_track' is active
        for term_name in env.termination_manager._term_names:
            if "collision" in term_name or "off_track" in term_name:
                collision_penalty += torch.where(env.termination_manager.get_term(term_name), 
                                               torch.tensor(-50.0, device=env.device), 
                                               torch.tensor(0.0, device=env.device))

    reward = lane_reward + speed_reward + collision_penalty
    
    # Handle NaNs
    nan_mask = torch.isnan(reward)
    if nan_mask.any():
        reward[nan_mask] = 0.0
        
    return reward

def lateral_error_reward(env: ManagerBasedRLEnv) -> torch.Tensor:
    """
    Reward based on lateral offset from track centerline.
    Original Logic: 
    - if abs(lat_err) < 0.5: +1.0
    - else: -abs(lat_err) * 2.0
    """
    obs = env.observation_manager.compute()["policy"]
    lat_err = obs[:, 8]
    
    reward = torch.where(
        torch.abs(lat_err) < 0.5,
        torch.ones_like(lat_err),
        -torch.abs(lat_err) * 2.0
    )
    
    # Handle NaNs
    nan_mask = torch.isnan(reward)
    if nan_mask.any():
        reward[nan_mask] = 0.0
    return reward

def line_penalty(env: ManagerBasedRLEnv) -> torch.Tensor:
    """Flat penalty if a white line was hit (handled by termination logic usually)."""
    # Check if white_line_contact termination was triggered
    hit = env.termination_manager.get_term("white_line_contact")
    reward = torch.where(hit, torch.tensor(-10.0, device=env.device), torch.tensor(0.0, device=env.device))
    return reward

def steering_jerk_penalty(env: ManagerBasedRLEnv) -> torch.Tensor:
    """Penalizes high steering actions for smoothness."""
    # Action index 0 is steering
    steering = env.action_manager.action[:, 0]
    reward = -0.1 * torch.abs(steering)
    return reward
