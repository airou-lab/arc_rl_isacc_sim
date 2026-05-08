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
    reward = torch.where(speed > 0, speed * 0.5, torch.tensor(-2.0, device=env.device))
    return reward

def termination_penalty(env: ManagerBasedRLEnv) -> torch.Tensor:
    """Large penalty triggered only when a termination occurs."""
    return torch.where(env.reset_buf, torch.tensor(-500.0, device=env.device), torch.tensor(0.0, device=env.device))

def lateral_error_reward(env: ManagerBasedRLEnv) -> torch.Tensor:
    """Reward based on lateral offset from track centerline."""
    lat_err = env.extras.get("lat_err", torch.zeros(env.num_envs, device=env.device))
    abs_lat = torch.abs(lat_err)
    threshold = 0.1
    reward = torch.where(abs_lat < threshold, torch.ones_like(lat_err), -abs_lat * 5.0)
    return reward

def line_penalty(env: ManagerBasedRLEnv) -> torch.Tensor:
    hit = env.termination_manager.get_term("white_line_contact")
    reward = torch.where(hit, torch.tensor(-10.0, device=env.device), torch.tensor(0.0, device=env.device))
    return reward

def action_rate_smoothness_reward(env: ManagerBasedRLEnv) -> torch.Tensor:
    current_action = env.action_manager.action
    if "prev_action" not in env.extras:
        return torch.zeros(env.num_envs, device=env.device)
    prev_action = env.extras["prev_action"]
    reward = -1.0 * torch.square(current_action[:, 0] - prev_action[:, 0])
    return reward

def heading_alignment_reward(env: ManagerBasedRLEnv) -> torch.Tensor:
    head_err = env.extras.get("head_err", torch.zeros(env.num_envs, device=env.device))
    return torch.cos(head_err)
