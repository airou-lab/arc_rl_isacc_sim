# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import torch
from isaaclab.managers import SceneEntityCfg
from isaaclab.envs import ManagerBasedRLEnv

def progress_reward(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    """
    Rewards the agent heavily for moving forward efficiently.
    Subtracts a baseline so creeping (e.g. < 0.5 m/s) yields little to no reward,
    forcing the agent to discover that real speed pays off.
    """
    asset = env.scene[asset_cfg.name]
    
    # The car's forward direction is -X in its local frame.
    local_vel = asset.data.root_lin_vel_b
    forward_speed = -local_vel[:, 0]
    
    # Baseline speed of 0.5 m/s. Anything below this gets 0 bonus.
    # Anything above scales linearly, capped at 3.0 m/s relative (so 3.5 m/s absolute).
    speed_bonus = torch.clamp((forward_speed - 0.5) * 5.0, min=0.0, max=15.0)
    
    # Keep a small baseline reward just for not being negative, but make the 
    # bonus the dominant signal.
    base_speed = torch.clamp(forward_speed, min=-1.0) * 2.0
    
    return base_speed + speed_bonus

def termination_penalty(env: ManagerBasedRLEnv) -> torch.Tensor:
    """Large penalty triggered only when a termination occurs."""
    return torch.where(env.reset_buf, torch.tensor(-500.0, device=env.device), torch.tensor(0.0, device=env.device))

def lateral_error_reward(env: ManagerBasedRLEnv) -> torch.Tensor:
    """
    Reward based on lateral offset from track centerline.
    Continuous Gradient: No plateau. Every mm closer to center increases reward.
    Zero reward at 0.10m, negative beyond.
    """
    lat_err = env.extras.get("lat_err", torch.zeros(env.num_envs, device=env.device))
    abs_lat = torch.abs(lat_err)
    
    # 1.0 at center (0.0m), 0.0 at 0.10m. Linear dropoff.
    reward = torch.clamp(1.0 - (abs_lat * 10.0), min=-1.0)
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
