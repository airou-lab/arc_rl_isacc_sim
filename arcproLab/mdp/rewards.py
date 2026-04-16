# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import torch
from isaaclab.managers import SceneEntityCfg
from isaaclab.envs import ManagerBasedRLEnv

def speed_reward(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    """Rewards forward speed, heavily penalizes reverse driving."""
    asset = env.scene[asset_cfg.name]
    speed = asset.data.root_lin_vel_b[:, 0]
    
    # Positive reward for forward speed, -2.0 penalty for reverse
    reward = torch.where(speed > 0, speed * 0.5, torch.tensor(-2.0, device=env.device))
    
    # Handle NaNs
    nan_mask = torch.isnan(reward)
    if nan_mask.any():
        reward[nan_mask] = 0.0
    return reward

def lateral_error_reward(env: ManagerBasedRLEnv) -> torch.Tensor:
    """Reward based on lateral offset from track centerline. Reads from env.extras."""
    lat_err = env.extras.get("lat_err", torch.zeros(env.num_envs, device=env.device))
    
    # Target: Lane Center (0.56m offset was used in Phase 1, but TrackManager already centers waypoints)
    # Since TrackManager centers waypoints, target is 0.0
    abs_lat = torch.abs(lat_err)
    
    # Calibration Threshold: 0.1m error from center
    threshold = 0.1
    
    reward = torch.where(
        abs_lat < threshold,
        torch.ones_like(lat_err),
        -abs_lat * 5.0 # Scale penalty
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

def action_rate_smoothness_reward(env: ManagerBasedRLEnv) -> torch.Tensor:
    """Penalizes rapid changes in actions (wiggling)."""
    # Current vs Previous actions
    current_action = env.action_manager.action
    
    # Check if prev_action is available
    if not hasattr(env.action_manager, "prev_action") or env.action_manager.prev_action is None:
        return torch.zeros(env.num_envs, device=env.device)
        
    prev_action = env.action_manager.prev_action
    
    # Penalize the squared difference (heavier penalty for large jumps)
    # Focus mainly on steering (index 0)
    steer_diff = current_action[:, 0] - prev_action[:, 0]
    reward = -1.0 * torch.square(steer_diff)
    
    return reward

def heading_alignment_reward(env: ManagerBasedRLEnv) -> torch.Tensor:
    """Rewards facing the correct way along the track waypoints. Reads from env.extras."""
    head_err = env.extras.get("head_err", torch.zeros(env.num_envs, device=env.device))
    
    # Reward is cosine of error (max 1.0 when perfectly aligned, negative if > 90deg)
    reward = torch.cos(head_err)
    
    # Handle NaNs
    nan_mask = torch.isnan(reward)
    if nan_mask.any():
        reward[nan_mask] = 0.0
    return reward
