# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import torch
from isaaclab.managers import SceneEntityCfg
from isaaclab.envs import ManagerBasedRLEnv

def speed_reward(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    """Rewards forward speed (current_speed * 2.0)."""
    extract = env.scene[asset_cfg.name]
    speed = extract.data.root_lin_vel_b[:, 0]
    base_reward = speed * 2.0
    return torch.where(env.episode_length_buf >= 20, base_reward, torch.zeros_like(base_reward))

def line_penalty(env: ManagerBasedRLEnv) -> torch.Tensor:
    """Applies a flat -100 penalty if a white line was hit this step."""
    # This logic checks the termination manager's status for the line contact term
    hit = env.termination_manager.get_term("white_line_contact")
    base_penalty = torch.where(hit, torch.tensor(-100.0, device=env.device), torch.tensor(0.0, device=env.device))
    return torch.where(env.episode_length_buf >= 20, base_penalty, torch.zeros_like(base_penalty))

def lateral_error_reward(env: ManagerBasedRLEnv) -> torch.Tensor:
    """
    Original Evolution Reward:
    - If abs(lateral_error) < 0.5: +1.0
    - Else: -abs(lateral_error) * 2.0
    
    Lateral error is at Index 8 of the telemetry vector.
    """
    # Pull the telemetry vector from the observation manager buffer
    # The observation manager 'compute()' returns the current observations for all groups
    obs = env.observation_manager.compute()["policy"]
    lat_err = obs[:, 8]
    
    base_reward = torch.where(
        torch.abs(lat_err) < 0.5,
        torch.ones_like(lat_err),
        -torch.abs(lat_err) * 2.0
    )
    return torch.where(env.episode_length_buf >= 20, base_reward, torch.zeros_like(base_reward))

def steering_jerk_penalty(env: ManagerBasedRLEnv) -> torch.Tensor:
    """Penalizes high-frequency steering changes."""
    # Action index 0 is steering
    delta_steer = env.action_manager.action[:, 0] - env.action_manager.prev_action[:, 0]
    base_penalty = -1.0 * torch.square(delta_steer)
    return torch.where(env.episode_length_buf >= 20, base_penalty, torch.zeros_like(base_penalty))
