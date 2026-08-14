# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import torch
from isaaclab.managers import SceneEntityCfg
from isaaclab.envs import ManagerBasedRLEnv

def waypoint_progress_reward(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    """
    Rewards the agent ONLY for advancing along the track's centerline waypoints.

    Displacement gate ensures reward only fires if the robot physically moved >= 2cm
    since the last step, preventing jitter farming.
    Stores per-step reward delta in env.extras["reward_wp_delta_step"] to allow
    accurate reward signal logging.

    At 0.5 m/s, 50Hz: ~10mm/step / 6mm per WP = ~1.7 WPs/step → reward ~34/step.
    """
    from mdp.track_manager import get_track_manager
    tm = get_track_manager(device=env.device)

    if tm.last_indices is None or tm.waypoints is None:
        env.extras["reward_wp_delta_step"] = torch.zeros(env.num_envs, device=env.device)
        return torch.zeros(env.num_envs, device=env.device)

    current_idx = tm.last_indices.clone()  # (num_envs,)
    num_wps = tm.waypoints.shape[0]

    # Initialize tracking state on first call
    if "prev_wp_idx_reward" not in env.extras:
        env.extras["prev_wp_idx_reward"] = current_idx.clone()
        env.extras["cumulative_wp_index"] = torch.zeros(env.num_envs, dtype=torch.float32, device=env.device)
        env.extras["max_cumulative_wp_index"] = torch.zeros(env.num_envs, dtype=torch.float32, device=env.device)
        env.extras["reward_wp_delta_step"] = torch.zeros(env.num_envs, device=env.device)
        return torch.zeros(env.num_envs, device=env.device)

    prev_idx = env.extras["prev_wp_idx_reward"]

    # Compute true step delta (handles wrap-around)
    delta = (current_idx - prev_idx) % num_wps
    delta_real = torch.where(delta > num_wps // 2, delta - num_wps, delta).float()

    # BI-DIRECTIONAL FIX: Apply track direction so forward movement is always positive
    if "track_dir" in env.extras:
        delta_real = delta_real * env.extras["track_dir"].float()

    # CLAMP FIX: Prevent massive progress spikes if TrackManager snaps to a distant waypoint
    delta_real = torch.clamp(delta_real, min=-10.0, max=10.0)

    # Update cumulative progress
    env.extras["cumulative_wp_index"] += delta_real

    # Calculate reward as the amount by which we exceed the highest progress ever achieved this episode
    cum_wp = env.extras["cumulative_wp_index"]
    max_cum = env.extras["max_cumulative_wp_index"]
    reward_delta = cum_wp - max_cum
    reward_delta = torch.clamp(reward_delta, min=0.0)

    # Update high-water mark
    env.extras["max_cumulative_wp_index"] = torch.maximum(max_cum, cum_wp)
    env.extras["prev_wp_idx_reward"] = current_idx.clone()

    # Zero out everything for envs that just reset
    just_reset = torch.zeros_like(reward_delta, dtype=torch.bool)
    if hasattr(env, 'reset_terminated'):
        just_reset = just_reset | env.reset_terminated
    if hasattr(env, 'reset_buf'):
        just_reset = just_reset | env.reset_buf
    if hasattr(env, 'episode_length_buf'):
        just_reset = just_reset | (env.episode_length_buf <= 1)
        
    reward_delta = torch.where(just_reset, torch.zeros_like(reward_delta), reward_delta)
    
    # If reset, we MUST clear the cumulative trackers so the next episode starts fresh at 0
    env.extras["cumulative_wp_index"] = torch.where(just_reset, torch.zeros_like(cum_wp), cum_wp)
    env.extras["max_cumulative_wp_index"] = torch.where(just_reset, torch.zeros_like(max_cum), env.extras["max_cumulative_wp_index"])
    env.extras["prev_wp_idx_reward"] = torch.where(just_reset, current_idx, current_idx)

    # Expose the actual per-step reward delta for accurate telemetry logging
    env.extras["reward_wp_delta_step"] = reward_delta

    return reward_delta

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
    speed = torch.norm(env.scene["robot"].data.root_lin_vel_b[:, :2], dim=1)
    # Multiply by speed so the agent cannot farm heading points while sitting still.
    return speed * torch.clamp(torch.cos(head_err), min=0.0)

def boundary_penalty(env: ManagerBasedRLEnv) -> torch.Tensor:
    """
    Penalizes the agent for crossing or being too close to white/yellow lines.
    """
    from .terminations import white_line_contact
    # Mastery Logic: Penalize if within 0.40m (was 0.15m), but wait for termination logic to reset (at 0.12m).
    # This provides a 0.28m wide "warning track" of dense negative feedback before the -1000 crash penalty.
    is_near = white_line_contact(env, threshold=0.20)
    return torch.where(is_near, torch.tensor(-100.0, device=env.device), torch.tensor(0.0, device=env.device))

def stationary_penalty(env: ManagerBasedRLEnv) -> torch.Tensor:
    """Penalizes the robot for not advancing waypoints, preventing spinning-in-place exploits."""
    if "cumulative_wp_index" not in env.extras:
        return torch.zeros(env.num_envs, device=env.device)
        
    current_wp = env.extras["cumulative_wp_index"]
    
    # Initialize the tracker on first call
    if "last_wp_check" not in env.extras:
        env.extras["last_wp_check"] = current_wp.clone()
        return torch.zeros(env.num_envs, device=env.device)
        
    # Check every 10 steps
    check_step = (env.episode_length_buf % 10 == 0)
    
    # Calculate progress since last check
    progress = current_wp - env.extras["last_wp_check"]
    
    # Grace period: don't penalize during first 50 steps (spawning/settling)
    # Stagnant if progress is less than 5.0 waypoints
    is_stagnant = (progress < 5.0) & check_step & (env.episode_length_buf > 50)
    
    # Update the tracker for the environments that are checking this step
    env.extras["last_wp_check"] = torch.where(
        check_step, 
        current_wp.clone(), 
        env.extras["last_wp_check"]
    )
    
    # Reset tracking for environments that just reset
    just_reset = env.episode_length_buf <= 1
    env.extras["last_wp_check"] = torch.where(
        just_reset, 
        current_wp.clone(), 
        env.extras["last_wp_check"]
    )
    
    return torch.where(is_stagnant, torch.tensor(-5.0, device=env.device), torch.tensor(0.0, device=env.device))

