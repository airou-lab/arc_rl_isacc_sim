# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import torch
from isaaclab.managers import SceneEntityCfg
from isaaclab.envs import ManagerBasedRLEnv

def progress_reward(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    """
    Rewards the agent linearly for forward distance traveled.
    Because dt is constant, returning forward_speed means Total Reward = Distance.
    This prevents the agent from farming 'time steps' by driving slowly before a crash.
    """
    asset = env.scene[asset_cfg.name]
    local_vel = asset.data.root_lin_vel_b
    forward_speed = local_vel[:, 0]
    
    # Simple linear reward: go forward, get points.
    return forward_speed

def termination_penalty(env: ManagerBasedRLEnv) -> torch.Tensor:
    """Penalty triggered only when a termination occurs."""
    return torch.where(env.reset_terminated, torch.tensor(-50.0, device=env.device), torch.tensor(0.0, device=env.device))

def lateral_error_reward(env: ManagerBasedRLEnv) -> torch.Tensor:
    """
    Penalty based on lateral offset from track centerline.
    0.0 at center, drops to negative values as it drifts.
    This prevents farming 'centering points' while standing still.
    """
    lat_err = env.extras.get("lat_err", torch.zeros(env.num_envs, device=env.device))
    abs_lat = torch.abs(lat_err)
    
    # Pure penalty
    reward = - (abs_lat * 10.0)
    return reward

def jerk_penalty(env: ManagerBasedRLEnv) -> torch.Tensor:
    """
    Penalizes high-frequency steering oscillations (Jitter).
    Weight: -100.0
    """
    if "prev_action" not in env.extras:
        return torch.zeros(env.num_envs, device=env.device)
    
    # Action index 0 is Steering
    steer_delta = env.action_manager.action[:, 0] - env.extras["prev_action"][:, 0]
    return -100.0 * torch.square(steer_delta)

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

def boundary_penalty(env: ManagerBasedRLEnv) -> torch.Tensor:
    """
    Penalizes the agent for crossing or being too close to white/yellow lines.
    Mastery Logic: Penalize if within 0.15m, but wait for termination logic to reset.
    """
    from .terminations import white_line_contact
    # Use a slightly more permissive check for the penalty itself
    # We reuse the logic but we can check if it's "close enough" to be a penalty
    is_near = white_line_contact(env, threshold=0.15)
    return torch.where(is_near, torch.tensor(-100.0, device=env.device), torch.tensor(0.0, device=env.device))
