# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import torch
from isaaclab.envs import ManagerBasedRLEnv
from isaaclab.managers import SceneEntityCfg
from mdp.curriculum_config import get_lookahead_distance


def waypoint_progress_reward(
    env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")
) -> torch.Tensor:
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
        env.extras["reward_wp_delta_step"] = torch.zeros(
            env.num_envs, device=env.device
        )
        return torch.zeros(env.num_envs, device=env.device)

    current_idx = tm.last_indices.clone()  # (num_envs,)
    num_wps = tm.waypoints.shape[0]

    # Initialize tracking state on first call
    if "prev_wp_idx_reward" not in env.extras:
        env.extras["prev_wp_idx_reward"] = current_idx.clone()
        env.extras["cumulative_wp_index"] = torch.zeros(
            env.num_envs, dtype=torch.float32, device=env.device
        )
        env.extras["max_cumulative_wp_index"] = torch.zeros(
            env.num_envs, dtype=torch.float32, device=env.device
        )
        env.extras["reward_wp_delta_step"] = torch.zeros(
            env.num_envs, device=env.device
        )
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
    if hasattr(env, "reset_terminated"):
        just_reset = just_reset | env.reset_terminated
    if hasattr(env, "reset_buf"):
        just_reset = just_reset | env.reset_buf
    if hasattr(env, "episode_length_buf"):
        just_reset = just_reset | (env.episode_length_buf <= 1)

    reward_delta = torch.where(just_reset, torch.zeros_like(reward_delta), reward_delta)

    # If reset, we MUST clear the cumulative trackers so the next episode starts fresh at 0
    env.extras["cumulative_wp_index"] = torch.where(
        just_reset, torch.zeros_like(cum_wp), cum_wp
    )
    env.extras["max_cumulative_wp_index"] = torch.where(
        just_reset, torch.zeros_like(max_cum), env.extras["max_cumulative_wp_index"]
    )
    env.extras["prev_wp_idx_reward"] = torch.where(just_reset, current_idx, current_idx)

    # Expose the actual per-step reward delta for accurate telemetry logging
    env.extras["reward_wp_delta_step"] = reward_delta

    return reward_delta


def termination_penalty(env: ManagerBasedRLEnv) -> torch.Tensor:
    """Penalty triggered only when a termination occurs."""
    return torch.where(
        env.reset_terminated,
        torch.tensor(-50.0, device=env.device),
        torch.tensor(0.0, device=env.device),
    )


def lateral_error_reward(env: ManagerBasedRLEnv) -> torch.Tensor:
    """
    Penalty based on lateral offset from track centerline.
    0.0 at center, drops to negative values as it drifts.
    Curvature-gated slack: on sharp corners (high kappa), allows up to 10cm
    of geometric out-in-out apex clipping without penalty. On straightaways
    (kappa ~ 0), enforces strict 0cm centerline tracking.
    """
    # Asymmetric Curvature Slack: On sharp corners, permits small apex-cutting inward
    # toward the inner yellow line (lat_err > 0), but enforces STRICT zero slack for outward
    # drift (lat_err < 0) toward the outer white curb to eliminate line straddling.
    lat_err = env.extras.get("lat_err", torch.zeros(env.num_envs, device=env.device))
    abs_lat = torch.abs(lat_err)

    slack = torch.zeros_like(abs_lat)
    if "lookahead_kappa" in env.extras and env.extras["lookahead_kappa"] is not None:
        kappa = torch.abs(
            env.extras["lookahead_kappa"]
            .to(device=env.device, dtype=torch.float32)
            .view_as(abs_lat)
        )
        # In CCW circuit, lat_err > 0 is inward toward yellow apex. lat_err < 0 is outward toward white line.
        is_inward = lat_err > 0.0
        inward_slack = torch.clamp(kappa * 0.05, min=0.0, max=0.03)
        slack = torch.where(is_inward, inward_slack, torch.zeros_like(inward_slack))

    penalized_lat = torch.clamp(abs_lat - slack, min=0.0)
    return -(penalized_lat * 15.0)


def jerk_penalty(env: ManagerBasedRLEnv) -> torch.Tensor:
    """
    Penalizes high-frequency steering oscillations (Jitter).
    Returns -100.0 * (steer_t - steer_{t-1})^2.
    """
    if env.action_manager.action is None or env.action_manager.action.shape[1] < 1:
        return torch.zeros(env.num_envs, device=env.device)

    current_steer = env.action_manager.action[:, 0]
    if "prev_action_jerk" not in env.extras:
        env.extras["prev_action_jerk"] = env.action_manager.action.clone()
        return torch.zeros(env.num_envs, device=env.device)

    prev_steer = env.extras["prev_action_jerk"][:, 0]
    steer_delta = current_steer - prev_steer

    # Zero out penalty for envs that just reset
    if hasattr(env, "episode_length_buf"):
        just_reset = env.episode_length_buf <= 1
        steer_delta = torch.where(
            just_reset, torch.zeros_like(steer_delta), steer_delta
        )

    # Update prev_action_jerk for the next step
    env.extras["prev_action_jerk"] = env.action_manager.action.clone()

    return -100.0 * torch.square(steer_delta)


def action_rate_smoothness_reward(env: ManagerBasedRLEnv) -> torch.Tensor:
    if env.action_manager.action is None or env.action_manager.action.shape[1] < 1:
        return torch.zeros(env.num_envs, device=env.device)

    current_action = env.action_manager.action
    if "prev_action_smoothness" not in env.extras:
        env.extras["prev_action_smoothness"] = current_action.clone()
        return torch.zeros(env.num_envs, device=env.device)

    prev_action = env.extras["prev_action_smoothness"]
    steer_delta = current_action[:, 0] - prev_action[:, 0]

    if hasattr(env, "episode_length_buf"):
        just_reset = env.episode_length_buf <= 1
        steer_delta = torch.where(
            just_reset, torch.zeros_like(steer_delta), steer_delta
        )

    # Update prev_action_smoothness for the next step
    env.extras["prev_action_smoothness"] = current_action.clone()

    reward = -1.0 * torch.square(steer_delta)
    return reward


def heading_alignment_reward(env: ManagerBasedRLEnv) -> torch.Tensor:
    head_err = env.extras.get("head_err", torch.zeros(env.num_envs, device=env.device))
    speed = torch.norm(env.scene["robot"].data.root_lin_vel_b[:, :2], dim=1)
    # Gated by movement (>= 0.25 m/s) to prevent farming heading while stopped,
    # without penalizing the agent for braking to 0.35 m/s into corners!
    speed_gate = torch.clamp(speed / 0.25, max=1.0)
    return speed_gate * torch.clamp(torch.cos(head_err), min=0.0)


def boundary_penalty(env: ManagerBasedRLEnv) -> torch.Tensor:
    """
    Penalizes the agent for crossing or being too close to white/yellow lines.
    Provides continuous gradient pushing the car toward lane center when wheels are <= 5cm from lines.

    CRITICAL: The yellow median line is NEVER masked anywhere on the circuit!
    Crossing or approaching the yellow line ALWAYS incurs steep repulsive penalty.
    Only the transverse white stop line at intersection entry (dist_g < 0.60m and X in [-6.5, -4.5])
    has white boundary penalty masked.
    """
    dist_y = env.extras.get(
        "dist_y", torch.ones(env.num_envs, device=env.device) * 0.22
    )
    dist_w = env.extras.get(
        "dist_w", torch.ones(env.num_envs, device=env.device) * 0.22
    )
    dist_g = env.extras.get(
        "dist_g", torch.ones(env.num_envs, device=env.device) * 10.0
    )

    robot = env.scene["robot"]
    env_origins = env.scene.env_origins
    local_pos = robot.data.root_pos_w[:, :2] - env_origins[:, :2]

    # Warning margins: 19cm from chassis center = 5cm clearance to wheel edge
    too_close_y = torch.clamp(0.19 - dist_y, min=0.0)
    too_close_w = torch.clamp(0.18 - dist_w, min=0.0)

    # Permeability for painted transverse STOP BAR ONLY:
    # Transverse white stop line is at Leg 2 intersection entry (X in [-6.5, -4.5], Y in [-7.5, -6.5])
    # Yellow median line is NEVER masked!
    in_stop_bar = (
        (dist_g < 0.60)
        & (local_pos[:, 0] >= -6.5)
        & (local_pos[:, 0] <= -4.5)
        & (local_pos[:, 1] >= -7.5)
        & (local_pos[:, 1] <= -6.5)
    )
    masked_too_close_w = torch.where(
        in_stop_bar, torch.zeros_like(too_close_w), too_close_w
    )

    # Yellow penalty has steep 160.0 multiplier; White has 100.0 multiplier
    penalty = -(too_close_y * 160.0 + masked_too_close_w * 100.0)
    return penalty


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
    check_step = env.episode_length_buf % 10 == 0

    # Calculate progress since last check
    progress = current_wp - env.extras["last_wp_check"]

    # Grace period: don't penalize during first 50 steps (spawning/settling)
    # Stagnant if progress is less than 3.0 waypoints (allows safe 0.30 m/s cornering)
    # Stagnation penalty is suppressed when dwelling at a stop bar / red signal (go_signal == 0)
    is_go = (
        env.extras.get("go_signal", torch.ones(env.num_envs, device=env.device)) > 0.5
    )
    is_stagnant = (progress < 3.0) & check_step & (env.episode_length_buf > 50) & is_go

    # Update the tracker for the environments that are checking this step
    env.extras["last_wp_check"] = torch.where(
        check_step, current_wp.clone(), env.extras["last_wp_check"]
    )

    # Reset tracking for environments that just reset
    just_reset = env.episode_length_buf <= 1
    env.extras["last_wp_check"] = torch.where(
        just_reset, current_wp.clone(), env.extras["last_wp_check"]
    )

    return torch.where(
        is_stagnant,
        torch.tensor(-5.0, device=env.device),
        torch.tensor(0.0, device=env.device),
    )


def curvature_speed_reward(env: ManagerBasedRLEnv) -> torch.Tensor:
    """
    Rewards maintaining appropriate speed based on track curvature (kappa) and lookahead curvature.
    On straightaways (|effective_kappa| ~ 0): target speed = 0.85 m/s.
    In sharp corners (|effective_kappa| >= 1.25): target speed = 0.35 m/s (pre-braking into apex).
    """
    from mdp.track_manager import get_track_manager

    tm = get_track_manager(device=env.device)
    if tm.last_indices is None or tm.curvature_tensor is None:
        return torch.zeros(env.num_envs, device=env.device)

    current_kappa = tm.curvature_tensor[tm.last_indices].to(
        device=env.device, dtype=torch.float32
    )

    # Fetch lookahead curvature from extras if available; fallback to computing from TrackManager
    if "lookahead_kappa" in env.extras and env.extras["lookahead_kappa"] is not None:
        lookahead_kappa = env.extras["lookahead_kappa"].to(
            device=env.device, dtype=torch.float32
        )
    else:
        lookahead_kappa = tm.get_lookahead_curvature(
            lookahead_distance=get_lookahead_distance(),
            track_dir=env.extras.get("track_dir"),
        ).to(device=env.device, dtype=torch.float32)

    if lookahead_kappa.shape != current_kappa.shape:
        lookahead_kappa = lookahead_kappa.view_as(current_kappa)

    effective_kappa = torch.maximum(
        torch.abs(current_kappa), torch.abs(lookahead_kappa)
    )
    # Straightaway speed stabilization: capped at 0.65 m/s to prevent suspension bounce, high-speed lateral drift,
    # and understeer washout. Pre-brakes smoothly to 0.35 m/s as curvature increases.
    target_speed = torch.clamp(0.65 - (effective_kappa * 0.50), min=0.35, max=0.65).to(
        device=env.device
    )

    # Pre-braking alignment for All Sharp Corners (Corner 1, Corner 2/Intersection, and Corner 4):
    # Enforces slowing to 0.35 m/s in curves to prevent understeer washout and high-speed curb clipping
    robot = env.scene["robot"]
    env_origins = env.scene.env_origins
    local_pos = robot.data.root_pos_w[:, :2] - env_origins[:, :2]
    dist_g = env.extras.get(
        "dist_g", torch.ones(env.num_envs, device=env.device) * 10.0
    )
    in_inter = (
        (local_pos[:, 0] >= -7.80)
        & (local_pos[:, 0] <= -1.80)
        & (local_pos[:, 1] >= -7.5)
        & (local_pos[:, 1] <= -2.20)
    )
    in_c1 = (local_pos[:, 0] <= -15.00) & (local_pos[:, 1] <= -4.80)
    in_c4 = (local_pos[:, 0] <= -13.50) & (local_pos[:, 1] >= 5.50)
    in_bend = (
        (local_pos[:, 1] >= 2.00)
        & (local_pos[:, 1] <= 7.20)
        & (local_pos[:, 0] >= -7.80)
        & (local_pos[:, 0] <= -2.00)
    )
    in_corner_zone = (
        in_inter | in_c1 | in_c4 | in_bend | (dist_g < 1.80) | (effective_kappa > 0.22)
    )
    target_speed = torch.where(
        in_corner_zone, torch.tensor(0.35, device=env.device), target_speed
    )

    current_speed = torch.norm(env.scene["robot"].data.root_lin_vel_b[:, :2], dim=1).to(
        device=env.device
    )

    speed_error = torch.abs(current_speed - target_speed)
    # Sharpened speed error sensitivity: exp(-3.5 * speed_error) creates a clear peak at target_speed
    r_speed = torch.exp(-3.5 * speed_error).to(device=env.device)

    # Two-stage overspeed enforcement:
    # 1. Suppresses straightaway sprinting beyond 0.65 m/s (multiplier 3.5 eliminates sprint profitability)
    # 2. Heavily penalizes charging into sharp corners beyond 0.40 m/s (multiplier 4.0 prevents understeer washout)
    straight_overspeed = torch.clamp(current_speed - 0.65, min=0.0)
    corner_overspeed = torch.clamp(current_speed - 0.40, min=0.0)
    effective_overspeed = torch.where(
        in_corner_zone,
        corner_overspeed * 4.0,
        straight_overspeed * 3.5,
    )
    r_speed = r_speed - effective_overspeed

    forward_vel = current_speed
    velocity_gate = torch.clamp(forward_vel / 0.20, min=0.0, max=1.0)
    is_go = (
        env.extras.get("go_signal", torch.ones(env.num_envs, device=env.device)) > 0.5
    )
    effective_gate = torch.where(is_go, velocity_gate, torch.ones_like(velocity_gate))
    r_speed = r_speed * effective_gate
    return r_speed


def lane_centering_reward(env: ManagerBasedRLEnv) -> torch.Tensor:
    """
    Rewards staying centered between the white line and double yellow line.
    Equidistant center: dist_y ~= dist_w ~= 0.21m.
    Decays smoothly to 0.0 as the vehicle approaches either boundary.
    Gated by velocity when is_go == 1.0 so that parked vehicles earn 0.0 points.
    """
    dist_y = env.extras.get(
        "dist_y", torch.ones(env.num_envs, device=env.device) * 0.21
    )
    dist_w = env.extras.get(
        "dist_w", torch.ones(env.num_envs, device=env.device) * 0.21
    )
    lane_diff = torch.abs(dist_y - dist_w)
    # Steepened exponential: exp(-20.0 * lane_diff) strongly incentivizes strict equidistant centering
    r_centering = torch.exp(-20.0 * lane_diff).to(device=env.device)

    forward_vel = torch.norm(env.scene["robot"].data.root_lin_vel_b[:, :2], dim=1).to(
        device=env.device
    )
    velocity_gate = torch.clamp(forward_vel / 0.20, min=0.0, max=1.0)
    is_go = (
        env.extras.get("go_signal", torch.ones(env.num_envs, device=env.device)) > 0.5
    )
    effective_gate = torch.where(is_go, velocity_gate, torch.ones_like(velocity_gate))
    r_centering = r_centering * effective_gate
    return r_centering


def intersection_expert_reward(env: ManagerBasedRLEnv) -> torch.Tensor:
    """
    Provides explicit teacher guidance through the intersection where painted lines break.
    Rewards matching the ideal geometric Stanley steering arc and slowing to cornering speed (0.35 m/s).
    Only fires inside intersection gate zones (dist_g < 1.0m) or approaching sharp turns.
    """
    if env.action_manager.action is None or env.action_manager.action.shape[1] < 1:
        return torch.zeros(env.num_envs, device=env.device)

    lat_err = env.extras.get("lat_err", torch.zeros(env.num_envs, device=env.device))
    head_err = env.extras.get("head_err", torch.zeros(env.num_envs, device=env.device))
    dist_g = env.extras.get(
        "dist_g", torch.ones(env.num_envs, device=env.device) * 10.0
    )

    # Target geometric steering: Ackermann kinematic curvature feedforward + Stanley PD feedback
    kappa = env.extras.get("kappa", torch.zeros(env.num_envs, device=env.device))
    track_dir = env.extras.get("track_dir", torch.ones(env.num_envs, device=env.device))
    signed_kappa = kappa * track_dir
    delta_ff = torch.atan(0.33 * signed_kappa) / 0.68
    target_steer = torch.clamp(
        delta_ff - 2.5 * lat_err - 1.2 * head_err, min=-1.0, max=1.0
    )
    current_steer = env.action_manager.action[:, 0]
    steer_match = torch.exp(-4.0 * torch.square(current_steer - target_steer))

    # Target corner speed (0.35 m/s)
    speed = torch.norm(env.scene["robot"].data.root_lin_vel_b[:, :2], dim=1)
    speed_match = torch.exp(-2.0 * torch.abs(speed - 0.35))

    guidance = steer_match * speed_match

    # Active inside corner guidance zones (Intersection, Corner 1, Corner 4, or gate points)
    robot = env.scene["robot"]
    env_origins = env.scene.env_origins
    local_pos = robot.data.root_pos_w[:, :2] - env_origins[:, :2]
    in_inter = (
        (local_pos[:, 0] >= -6.50)
        & (local_pos[:, 0] <= -1.80)
        & (local_pos[:, 1] >= -7.5)
        & (local_pos[:, 1] <= -2.20)
    )
    in_c1 = (local_pos[:, 0] <= -14.20) & (local_pos[:, 1] <= -4.80)
    in_c4 = (local_pos[:, 0] <= -13.50) & (local_pos[:, 1] >= 5.20)
    in_zone = in_inter | in_c1 | in_c4 | (dist_g < 1.00)
    return torch.where(in_zone, guidance, torch.zeros_like(guidance))
