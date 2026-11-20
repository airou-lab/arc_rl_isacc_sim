# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import sys
import torch

try:
    if "omni" not in sys.modules:
        raise ImportError
    from isaaclab.envs import ManagerBasedRLEnv
    from isaaclab.managers import SceneEntityCfg
except ImportError:
    ManagerBasedRLEnv = object  # type: ignore[misc,assignment]

    class SceneEntityCfg:  # type: ignore[no-redef]
        def __init__(self, name: str = "robot"):
            self.name = name

try:
    from mdp.curriculum_config import get_lookahead_distance
    from mdp.track_topology import (
        BOUNDARY_WARN_MARGIN,
        FLARED_LANE_WIDTH_THRESH,
        compute_step_road_state,
        in_any_corner,
        in_branch_permeable_gate,
        in_c2_turn,
        in_intersection_expert_zone,
        in_multi_branch_intersection,
        in_stop_bar,
        in_wrong_lane_intersection,
    )
except ImportError:
    from arcproLab.mdp.curriculum_config import get_lookahead_distance
    from arcproLab.mdp.track_topology import (
        BOUNDARY_WARN_MARGIN,
        FLARED_LANE_WIDTH_THRESH,
        compute_step_road_state,
        in_any_corner,
        in_branch_permeable_gate,
        in_c2_turn,
        in_intersection_expert_zone,
        in_multi_branch_intersection,
        in_stop_bar,
        in_wrong_lane_intersection,
    )


def _get_local_xy(env: ManagerBasedRLEnv) -> tuple[torch.Tensor, torch.Tensor]:
    """Retrieve robot local (x, y) coordinates from cached road state or scene."""
    if hasattr(env, "extras") and env.extras is not None and "local_pos" in env.extras:
        lp = env.extras["local_pos"]
        return lp[:, 0], lp[:, 1]
    try:
        robot = env.scene["robot"]
        env_origins = env.scene.env_origins
        local_pos = robot.data.root_pos_w[:, :2] - env_origins[:, :2]
        return local_pos[:, 0], local_pos[:, 1]
    except Exception:
        zeros = torch.zeros(env.num_envs, device=env.device, dtype=torch.float32)
        return zeros, zeros


def _ensure_road_state_if_empty(env: ManagerBasedRLEnv) -> None:
    """Populate road state cache if neither TerminationManager nor test fixtures populated extras."""
    if not hasattr(env, "extras") or env.extras is None:
        env.extras = {}
    if env.extras.get("_road_state_cache_valid", False):
        return
    if (
        "lat_err" not in env.extras
        and "dist_y" not in env.extras
        and "kappa" not in env.extras
        and "go_signal" not in env.extras
    ):
        compute_step_road_state(env)


def waypoint_progress_reward(
    env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")
) -> torch.Tensor:
    """
    Rewards the agent ONLY for advancing along the track's active centerline waypoints.

    Stores per-step reward delta in env.extras["reward_wp_delta_step"] to allow
    accurate reward signal logging.
    """
    try:
        from mdp.track_manager import get_track_manager
    except ImportError:
        from arcproLab.mdp.track_manager import get_track_manager

    tm = get_track_manager(device=env.device)

    if tm.last_indices is None or tm.waypoints is None:
        env.extras["reward_wp_delta_step"] = torch.zeros(
            env.num_envs, device=env.device
        )
        return torch.zeros(env.num_envs, device=env.device)

    current_idx = tm.last_indices.clone()
    num_wps = tm.waypoints.shape[0]
    current_branch = env.extras.get("active_branch_ids", None)
    if (
        not isinstance(current_branch, torch.Tensor)
        or current_branch.shape != current_idx.shape
    ):
        current_branch = (
            tm.last_branch_ids.clone()
            if isinstance(getattr(tm, "last_branch_ids", None), torch.Tensor)
            and tm.last_branch_ids.shape == current_idx.shape
            else torch.zeros_like(current_idx)
        )

    # Initialize tracking state on first call
    if (
        "prev_wp_idx_reward" not in env.extras
        or env.extras["prev_wp_idx_reward"].shape != current_idx.shape
    ):
        env.extras["prev_wp_idx_reward"] = current_idx.clone()
        env.extras["prev_branch_id_reward"] = current_branch.clone()
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
    prev_branch = env.extras.get("prev_branch_id_reward", None)

    # Re-anchor prev_idx whenever an environment switched active branches since the last reward step
    branch_switched = env.extras.get("branch_switched_mask", None)
    switched = torch.zeros_like(current_idx, dtype=torch.bool)
    if (
        isinstance(branch_switched, torch.Tensor)
        and branch_switched.shape == current_idx.shape
    ):
        switched = switched | branch_switched
    if (
        isinstance(prev_branch, torch.Tensor)
        and prev_branch.shape == current_branch.shape
    ):
        switched = switched | (current_branch != prev_branch)
    if switched.any():
        prev_idx = torch.where(switched, current_idx, prev_idx)

    # Compute true step delta (handles wrap-around)
    delta = (current_idx - prev_idx) % num_wps
    delta_real = torch.where(delta > num_wps // 2, delta - num_wps, delta).float()

    # Apply track direction so forward movement is always positive
    if "track_dir" in env.extras:
        delta_real = delta_real * env.extras["track_dir"].float()

    # Prevent progress spikes if TrackManager snaps to a distant waypoint
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
    env.extras["prev_branch_id_reward"] = current_branch.clone()

    # Zero out everything for envs that just reset
    just_reset = torch.zeros_like(reward_delta, dtype=torch.bool)
    if hasattr(env, "reset_terminated"):
        just_reset = just_reset | env.reset_terminated
    if hasattr(env, "reset_buf"):
        just_reset = just_reset | env.reset_buf
    if hasattr(env, "episode_length_buf"):
        just_reset = just_reset | (env.episode_length_buf <= 1)

    reward_delta = torch.where(just_reset, torch.zeros_like(reward_delta), reward_delta)

    # Clear cumulative trackers for reset envs so next episode starts fresh at 0
    env.extras["cumulative_wp_index"] = torch.where(
        just_reset, torch.zeros_like(cum_wp), cum_wp
    )
    env.extras["max_cumulative_wp_index"] = torch.where(
        just_reset, torch.zeros_like(max_cum), env.extras["max_cumulative_wp_index"]
    )
    env.extras["prev_wp_idx_reward"] = torch.where(just_reset, current_idx, current_idx)
    env.extras["prev_branch_id_reward"] = torch.where(
        just_reset, current_branch, current_branch
    )

    env.extras["reward_wp_delta_step"] = reward_delta
    return reward_delta


def termination_penalty(env: ManagerBasedRLEnv) -> torch.Tensor:
    """One-shot crash penalty scaled inversely by `step_dt` to neutralize RewardManager `dt` multiplication.

    At `step_dt = 0.02` and `weight = 400.0`, returns `-50.0` on termination,
    producing an exact `-400.0` effective terminal penalty.
    """
    return -(1.0 / max(float(getattr(env, "step_dt", 0.02)), 1e-6)) * env.reset_terminated.float()


def lateral_error_reward(env: ManagerBasedRLEnv) -> torch.Tensor:
    """
    Penalty based on lateral offset from track centerline (`-penalized_lat`,
    paired with `RewardCfg.lateral_error.weight = 225.0`).
    """
    _ensure_road_state_if_empty(env)
    lat_err = env.extras.get("lat_err", torch.zeros(env.num_envs, device=env.device))
    abs_lat = torch.abs(lat_err)

    slack = torch.zeros_like(abs_lat)
    if "kappa" in env.extras and env.extras["kappa"] is not None:
        signed_kappa = (
            env.extras["kappa"]
            .to(device=env.device, dtype=torch.float32)
            .view_as(abs_lat)
        )
        kappa = torch.abs(signed_kappa)
        # In left turns (kappa >= 0), lat_err > 0 is inward apex; in right turns (kappa < 0), lat_err < 0 is inward apex.
        is_inward = torch.where(signed_kappa < -1e-4, lat_err < 0.0, lat_err > 0.0)
        inward_slack = torch.clamp(kappa * 0.05, min=0.0, max=0.03)
        slack = torch.where(is_inward, inward_slack, torch.zeros_like(inward_slack))

    penalized_lat = torch.clamp(abs_lat - slack, min=0.0)
    return -penalized_lat


def jerk_penalty(env: ManagerBasedRLEnv) -> torch.Tensor:
    """
    Penalizes high-frequency steering oscillations (Jitter).
    Returns -100.0 * (steer_t - steer_{t-1})^2.
    """
    if env.action_manager.action is None or env.action_manager.action.shape[1] < 1:
        return torch.zeros(env.num_envs, device=env.device)

    current_steer = env.action_manager.action[:, 0]
    if (
        "prev_action_jerk" not in env.extras
        or env.extras["prev_action_jerk"].shape != env.action_manager.action.shape
    ):
        env.extras["prev_action_jerk"] = env.action_manager.action.clone()
        return torch.zeros(env.num_envs, device=env.device)

    prev_steer = env.extras["prev_action_jerk"][:, 0]
    steer_delta = current_steer - prev_steer

    if hasattr(env, "episode_length_buf"):
        just_reset = env.episode_length_buf <= 1
        steer_delta = torch.where(
            just_reset, torch.zeros_like(steer_delta), steer_delta
        )

    env.extras["prev_action_jerk"] = env.action_manager.action.clone()
    return -100.0 * torch.square(steer_delta)


def action_rate_smoothness_reward(env: ManagerBasedRLEnv) -> torch.Tensor:
    if env.action_manager.action is None or env.action_manager.action.shape[1] < 1:
        return torch.zeros(env.num_envs, device=env.device)

    current_action = env.action_manager.action
    if (
        "prev_action_smoothness" not in env.extras
        or env.extras["prev_action_smoothness"].shape != current_action.shape
    ):
        env.extras["prev_action_smoothness"] = current_action.clone()
        return torch.zeros(env.num_envs, device=env.device)

    prev_action = env.extras["prev_action_smoothness"]
    steer_delta = current_action[:, 0] - prev_action[:, 0]

    if hasattr(env, "episode_length_buf"):
        just_reset = env.episode_length_buf <= 1
        steer_delta = torch.where(
            just_reset, torch.zeros_like(steer_delta), steer_delta
        )

    env.extras["prev_action_smoothness"] = current_action.clone()
    return -1.0 * torch.square(steer_delta)


def heading_alignment_reward(env: ManagerBasedRLEnv) -> torch.Tensor:
    _ensure_road_state_if_empty(env)
    head_err = env.extras.get("head_err", torch.zeros(env.num_envs, device=env.device))
    speed = torch.norm(env.scene["robot"].data.root_lin_vel_b[:, :2], dim=1)
    speed_gate = torch.clamp(speed / 0.25, max=1.0)
    return speed_gate * torch.clamp(torch.cos(head_err), min=0.0)


def boundary_penalty(env: ManagerBasedRLEnv) -> torch.Tensor:
    """
    Penalizes the agent symmetrically for crossing or being too close to white/yellow lines
    using `BOUNDARY_WARN_MARGIN = 0.170` (zero centerline violations across all 5,000 WPs).

    The yellow median line is NEVER masked anywhere on the circuit.
    The transverse painted white stop bar and active multi-branch white exits have white penalty masked.
    """
    _ensure_road_state_if_empty(env)
    dist_y = env.extras.get(
        "dist_y", torch.ones(env.num_envs, device=env.device) * 0.21
    )
    dist_w = env.extras.get(
        "dist_w", torch.ones(env.num_envs, device=env.device) * 0.21
    )
    dist_g = env.extras.get(
        "dist_g", torch.ones(env.num_envs, device=env.device) * 10.0
    )

    x, y = _get_local_xy(env)

    too_close_y = torch.clamp(BOUNDARY_WARN_MARGIN - dist_y, min=0.0)
    too_close_w = torch.clamp(BOUNDARY_WARN_MARGIN - dist_w, min=0.0)

    branch_token = env.extras.get(
        "branch_token", env.extras.get("turn_token", None)
    )
    multi_branch_active = bool(env.extras.get("multi_branch_active", False))
    permeable_white_mask = in_branch_permeable_gate(
        x, y, dist_g, turn_token=branch_token, multi_branch_active=multi_branch_active
    )
    masked_too_close_w = torch.where(
        permeable_white_mask, torch.zeros_like(too_close_w), too_close_w
    )

    penalty = -(too_close_y * 160.0 + masked_too_close_w * 160.0)
    return penalty


def stationary_penalty(env: ManagerBasedRLEnv) -> torch.Tensor:
    """Penalizes the robot for not advancing waypoints, while bypassing penalty at red lights."""
    if "cumulative_wp_index" not in env.extras:
        return torch.zeros(env.num_envs, device=env.device)

    current_wp = env.extras["cumulative_wp_index"]

    if (
        "last_wp_check" not in env.extras
        or env.extras["last_wp_check"].shape != current_wp.shape
    ):
        env.extras["last_wp_check"] = current_wp.clone()
        return torch.zeros(env.num_envs, device=env.device)

    check_step = env.episode_length_buf % 10 == 0
    progress = current_wp - env.extras["last_wp_check"]

    go_signal = env.extras.get(
        "go_signal", torch.ones(env.num_envs, device=env.device)
    )
    is_go = go_signal > 0.5
    x, y = _get_local_xy(env)
    dist_g = env.extras.get(
        "dist_g", torch.ones(env.num_envs, device=env.device) * 10.0
    )
    holding_at_red = (~is_go) | (in_stop_bar(x, y, dist_g) & (~is_go))

    is_stagnant = (
        (progress < 3.0)
        & check_step
        & (env.episode_length_buf > 50)
        & is_go
        & (~holding_at_red)
    )

    env.extras["last_wp_check"] = torch.where(
        check_step | holding_at_red, current_wp.clone(), env.extras["last_wp_check"]
    )

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
    Rewards maintaining appropriate speed based on track curvature (kappa), lookahead curvature,
    and stop-bar traffic signals (`go_signal`).
    When `go_signal <= 0.5`, sets `target_speed = 0.0` (bypassing the 0.35 m/s corner override)
    so stopping cleanly at a red light earns full positive reward.
    """
    _ensure_road_state_if_empty(env)
    try:
        from mdp.track_manager import get_track_manager
    except ImportError:
        from arcproLab.mdp.track_manager import get_track_manager

    tm = get_track_manager(device=env.device)

    if "kappa" in env.extras and env.extras["kappa"] is not None:
        current_kappa = env.extras["kappa"].to(device=env.device, dtype=torch.float32)
    elif tm.last_indices is not None and tm.curvature_tensor is not None:
        current_kappa = tm.curvature_tensor[tm.last_indices].to(
            device=env.device, dtype=torch.float32
        )
    else:
        return torch.zeros(env.num_envs, device=env.device)

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

    dist_y_speed = env.extras.get(
        "dist_y", torch.ones(env.num_envs, device=env.device) * 0.21
    ).view_as(effective_kappa)
    head_err_speed = env.extras.get(
        "head_err", torch.zeros(env.num_envs, device=env.device)
    ).view_as(effective_kappa)

    # Symmetric/clean centering gate: full 1.0 at dist_y >= 0.180m (accommodating 0.406m lane width)
    centering_gate = torch.clamp((dist_y_speed - 0.145) / 0.035, min=0.0, max=1.0)
    heading_gate = torch.clamp(
        1.0 - (torch.abs(head_err_speed) / 0.20), min=0.0, max=1.0
    )
    alignment_factor = centering_gate * heading_gate

    base_straight = torch.clamp(0.65 - (effective_kappa * 0.50), min=0.35, max=0.65)
    target_speed = (0.35 + (base_straight - 0.35) * alignment_factor).to(
        device=env.device
    )

    x, y = _get_local_xy(env)
    in_corner_zone = in_any_corner(x, y) | (effective_kappa > 0.22)
    target_speed = torch.where(
        in_corner_zone, torch.tensor(0.35, device=env.device), target_speed
    )

    # Red-light override: when go_signal <= 0.5, target_speed = 0.0 (bypasses corner 0.35 override)
    go_signal = env.extras.get(
        "go_signal", torch.ones(env.num_envs, device=env.device)
    ).view_as(target_speed)
    is_go = go_signal > 0.5
    target_speed = torch.where(
        is_go, target_speed, torch.zeros_like(target_speed)
    )

    current_speed = torch.norm(env.scene["robot"].data.root_lin_vel_b[:, :2], dim=1).to(
        device=env.device
    )

    speed_error = torch.abs(current_speed - target_speed)
    r_speed = torch.exp(-3.5 * speed_error).to(device=env.device)

    straight_overspeed = torch.clamp(current_speed - 0.65, min=0.0)
    corner_overspeed = torch.clamp(current_speed - 0.40, min=0.0)
    red_light_overspeed = torch.clamp(current_speed - 0.10, min=0.0)
    effective_overspeed = torch.where(
        ~is_go,
        red_light_overspeed * 4.0,
        torch.where(
            in_corner_zone,
            corner_overspeed * 4.0,
            straight_overspeed * 3.5,
        ),
    )
    r_speed = r_speed - effective_overspeed

    velocity_gate = torch.clamp(current_speed / 0.20, min=0.0, max=1.0)
    effective_gate = torch.where(is_go, velocity_gate, torch.ones_like(velocity_gate))
    return r_speed * effective_gate


def lane_centering_reward(env: ManagerBasedRLEnv) -> torch.Tensor:
    """
    Rewards staying centered in the lane while eliminating non-flared static width offset
    tug-of-war:
      - When `(dist_y + dist_w <= 0.45) & (~in_c2_turn(x, y))`:
        `lane_diff = torch.minimum(torch.abs(dist_y - dist_w), 2.0 * torch.abs(lat_err))`
      - When flared (`dist_y + dist_w > 0.45`) or inside intersection (`in_c2_turn(x, y)`):
        `lane_diff = 2.0 * torch.abs(lat_err)`
    """
    _ensure_road_state_if_empty(env)
    dist_y = env.extras.get(
        "dist_y", torch.ones(env.num_envs, device=env.device) * 0.21
    )
    dist_w = env.extras.get(
        "dist_w", torch.ones(env.num_envs, device=env.device) * 0.21
    )
    lat_err = env.extras.get("lat_err", torch.zeros(env.num_envs, device=env.device))

    x, y = _get_local_xy(env)
    centerline_diff = 2.0 * torch.abs(lat_err)
    boundary_diff = torch.abs(dist_y - dist_w)

    branch_token = env.extras.get(
        "branch_token", env.extras.get("turn_token", None)
    )
    multi_branch_active = bool(env.extras.get("multi_branch_active", False))
    in_inter_zone = in_c2_turn(x, y) | (
        in_multi_branch_intersection(x, y)
        if multi_branch_active
        else torch.zeros_like(lat_err, dtype=torch.bool)
    )
    non_flared = ((dist_y + dist_w) <= FLARED_LANE_WIDTH_THRESH) & (~in_inter_zone)
    lane_diff = torch.where(
        non_flared,
        torch.minimum(boundary_diff, centerline_diff),
        centerline_diff,
    )

    r_centering = (
        0.6 * torch.exp(-15.0 * lane_diff) + 0.4 * torch.exp(-4.0 * lane_diff)
    ).to(device=env.device)

    in_wrong_lane = in_wrong_lane_intersection(
        x, y, turn_token=branch_token, multi_branch_active=multi_branch_active
    )
    r_centering = torch.where(
        in_wrong_lane, torch.tensor(-1.0, device=env.device), r_centering
    )

    forward_vel = torch.norm(env.scene["robot"].data.root_lin_vel_b[:, :2], dim=1).to(
        device=env.device
    )
    velocity_gate = torch.clamp(forward_vel / 0.20, min=0.0, max=1.0)
    is_go = (
        env.extras.get("go_signal", torch.ones(env.num_envs, device=env.device)) > 0.5
    )
    effective_gate = torch.where(is_go, velocity_gate, torch.ones_like(velocity_gate))
    return r_centering * effective_gate


def intersection_expert_reward(env: ManagerBasedRLEnv) -> torch.Tensor:
    """
    Provides explicit teacher guidance through corners and the intersection using
    Ackermann kinematic curvature feedforward + Stanley PD feedback.
    When `go_signal <= 0.5`, targets 0.0 m/s instead of 0.35 m/s so stopping at a red light
    is rewarded rather than penalized.
    """
    if env.action_manager.action is None or env.action_manager.action.shape[1] < 1:
        return torch.zeros(env.num_envs, device=env.device)

    _ensure_road_state_if_empty(env)
    lat_err = env.extras.get("lat_err", torch.zeros(env.num_envs, device=env.device))
    head_err = env.extras.get("head_err", torch.zeros(env.num_envs, device=env.device))
    kappa = env.extras.get("kappa", torch.zeros(env.num_envs, device=env.device))

    delta_ff = torch.atan(0.33 * kappa) / 0.68
    target_steer = torch.clamp(
        delta_ff - 2.5 * lat_err - 1.2 * head_err, min=-1.0, max=1.0
    )
    current_steer = env.action_manager.action[:, 0]
    steer_match = torch.exp(-12.0 * torch.square(current_steer - target_steer))

    speed = torch.norm(env.scene["robot"].data.root_lin_vel_b[:, :2], dim=1)
    is_go = (
        env.extras.get("go_signal", torch.ones(env.num_envs, device=env.device)) > 0.5
    )
    target_corner_v = torch.where(
        is_go, torch.full_like(speed, 0.35), torch.zeros_like(speed)
    )
    speed_match = torch.exp(-2.0 * torch.abs(speed - target_corner_v))

    guidance = steer_match * speed_match

    x, y = _get_local_xy(env)
    multi_branch_active = bool(env.extras.get("multi_branch_active", False))
    in_zone = (
        in_any_corner(x, y)
        | in_intersection_expert_zone(x, y)
        | (
            in_multi_branch_intersection(x, y)
            if multi_branch_active
            else torch.zeros_like(speed, dtype=torch.bool)
        )
    )

    straight_guidance = torch.exp(-25.0 * torch.square(current_steer - target_steer))
    # At a red stop bar outside or inside the intersection, reward stopping + steering alignment
    red_stop_guidance = steer_match * speed_match
    active_guidance = torch.where(
        ~is_go,
        red_stop_guidance,
        torch.where(in_zone, guidance, straight_guidance),
    )

    velocity_gate = torch.clamp(speed / 0.20, min=0.0, max=1.0)
    effective_gate = torch.where(is_go, velocity_gate, torch.ones_like(velocity_gate))
    return active_guidance * effective_gate


def straightaway_zero_steer_penalty(env: ManagerBasedRLEnv) -> torch.Tensor:
    """
    Smoothly penalizes non-zero steering on straightaways weighted by lane centering
    (`center_weight = exp(-(lat_err / 0.04)^2)`).
    Strictly gated by `~in_corner_zone` (`~(in_any_corner(x, y) | high_curvature | is_turn_token)`)
    so straightaway spawns receive smooth zero-steer regularization near the centerline while
    corrective steering when displaced laterally smoothly attenuates without a hard cutoff.
    """
    if env.action_manager.action is None or env.action_manager.action.shape[1] < 1:
        return torch.zeros(env.num_envs, device=env.device)

    _ensure_road_state_if_empty(env)
    current_steer = env.action_manager.action[:, 0]
    lat_err = env.extras.get("lat_err", torch.zeros(env.num_envs, device=env.device))
    kappa = env.extras.get("kappa", torch.zeros(env.num_envs, device=env.device))

    x, y = _get_local_xy(env)
    high_curvature = torch.abs(kappa) > 0.15

    turn_token = env.extras.get("turn_token", None)
    if turn_token is None:
        try:
            try:
                from mdp.road_manager import get_road_manager
            except ImportError:
                from arcproLab.mdp.road_manager import get_road_manager

            rm = get_road_manager(
                num_envs=env.num_envs, num_agents=1, device=env.device
            )
            turn_tokens, _ = rm.get_nav_commands()
            turn_token = turn_tokens.squeeze(1)
        except Exception:
            turn_token = torch.zeros(env.num_envs, device=env.device)

    is_turn_token = torch.abs(turn_token) >= 0.1
    in_corner_zone = in_any_corner(x, y) | high_curvature | is_turn_token

    in_straight = ~in_corner_zone
    center_weight = torch.exp(-torch.square(lat_err / 0.04))

    return torch.where(
        in_straight,
        -torch.square(current_steer) * center_weight,
        torch.zeros_like(current_steer),
    )


import sys as _sys  # noqa: E402

_sys.modules.setdefault("mdp.rewards", _sys.modules[__name__])
_sys.modules.setdefault("arcproLab.mdp.rewards", _sys.modules[__name__])

