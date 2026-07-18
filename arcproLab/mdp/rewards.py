# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import torch
from isaaclab.managers import SceneEntityCfg
from isaaclab.envs import ManagerBasedRLEnv

def progress_reward(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    """
    DEPRECATED: Rewards local body velocity. Can be gamed by spinning in circles.
    Kept for backward compatibility. Use waypoint_progress_reward instead.
    """
    asset = env.scene[asset_cfg.name]
    local_vel = asset.data.root_lin_vel_b
    forward_speed = local_vel[:, 0]
    return forward_speed

def waypoint_progress_reward(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    """
    Rewards the agent ONLY for advancing along the track's centerline waypoints.

    Fix C: Displacement gate — reward only fires if robot physically moved >= 2cm
    since the last step. This prevents jitter farming via windowed-search index
    oscillation (the nearest WP can snap ±1 even when the robot is spinning in place).

    Fix B: Stores per-step reward delta in env.extras["reward_wp_delta_step"] so
    train_skrl.py can log the actual reward signal, not the unrelated cumulative obs var.

    At 0.5 m/s, 50Hz: ~10mm/step / 6mm per WP = ~1.7 WPs/step → reward ~34/step.
    """
    from mdp.track_manager import get_track_manager
    tm = get_track_manager(device=env.device)

    if tm.last_indices is None or tm.waypoints is None:
        env.extras["reward_wp_delta_step"] = torch.zeros(env.num_envs, device=env.device)
        return torch.zeros(env.num_envs, device=env.device)

    asset = env.scene[asset_cfg.name]
    current_pos = asset.data.root_pos_w[:, :2].clone()  # (num_envs, 2) world XY
    current_idx = tm.last_indices  # (num_envs,)
    num_wps = tm.waypoints.shape[0]

    # Initialize tracking state on first call
    if "prev_wp_idx_reward" not in env.extras:
        env.extras["prev_wp_idx_reward"] = current_idx.clone()
        env.extras["prev_pos_reward"] = current_pos.clone()
        env.extras["reward_wp_delta_step"] = torch.zeros(env.num_envs, device=env.device)
        return torch.zeros(env.num_envs, device=env.device)

    prev_idx = env.extras["prev_wp_idx_reward"]
    prev_pos = env.extras["prev_pos_reward"]

    # Compute forward WP delta (handle wrap-around for closed tracks)
    delta = (current_idx - prev_idx) % num_wps

    # Clamp: if delta > half the track, agent went backward — no reward
    delta = torch.where(delta > num_wps // 2, torch.zeros_like(delta), delta)

    # FIX C — Displacement gate: only reward if robot actually moved >= 2cm.
    # Eliminates jitter farming caused by windowed-search WP index oscillation.
    displacement = torch.norm(current_pos - prev_pos, dim=1)  # (num_envs,)
    delta = torch.where(displacement >= 0.02, delta, torch.zeros_like(delta))

    # Zero out reward for envs that just terminated (prevent cross-episode delta bleed)
    if hasattr(env, 'reset_terminated'):
        delta = torch.where(env.reset_terminated, torch.zeros_like(delta), delta)
        # Reset tracking state for terminated envs
        env.extras["prev_wp_idx_reward"] = torch.where(
            env.reset_terminated, current_idx, prev_idx
        )
        env.extras["prev_pos_reward"] = torch.where(
            env.reset_terminated.unsqueeze(1).expand_as(current_pos),
            current_pos, prev_pos
        )
    else:
        env.extras["prev_wp_idx_reward"] = current_idx.clone()
        env.extras["prev_pos_reward"] = current_pos.clone()

    # FIX B — Expose the actual per-step reward delta for accurate telemetry logging
    env.extras["reward_wp_delta_step"] = delta.float()

    return delta.float()

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
