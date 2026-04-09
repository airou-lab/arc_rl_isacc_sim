# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import torch
from isaaclab.managers import SceneEntityCfg
from isaaclab.envs import ManagerBasedRLEnv

def speed_reward(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    """Rewards forward speed (current_speed * 0.3)."""
    asset = env.scene[asset_cfg.name]
    speed = asset.data.root_lin_vel_b[:, 0]
    
    # Simple speed reward (matching the logic from original policy environment)
    reward = speed * 0.3
    
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
    
    # Calibration Threshold: 2.0m physical error
    # 2.0m * 0.125 = 0.25 normalized
    threshold = 0.25
    
    reward = torch.where(
        torch.abs(lat_err) < threshold,
        torch.ones_like(lat_err),
        -torch.abs(lat_err) * 5.0 # Increased penalty for crossing boundaries
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
