# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import math
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
    from mdp.track_topology import (
        BOUNDARY_CONTACT_WHITE_CORNER,
        BOUNDARY_CONTACT_WHITE_STRAIGHT,
        BOUNDARY_CONTACT_YELLOW_CORNER,
        BOUNDARY_CONTACT_YELLOW_STRAIGHT,
        BOUNDARY_CORNER_KAPPA_THRESH,
        MAX_LAT_ERR_DEFAULT,
        MAX_LAT_ERR_INTERSECTION,
        OFF_ROAD_SHOULDER_DIST_Y,
        OFF_ROAD_SHOULDER_LAT_ERR,
        compute_step_road_state,
        in_any_corner,
        in_bend_turn,
        in_branch_permeable_gate,
        in_c1_turn,
        in_c2_turn,
        in_c4_turn,
        in_intersection_expert_zone,
        in_multi_branch_intersection,
        in_off_cliff_east,
        in_stop_bar,
        in_wrong_lane_intersection,
    )
except ImportError:
    from arcproLab.mdp.track_topology import (
        BOUNDARY_CONTACT_WHITE_CORNER,
        BOUNDARY_CONTACT_WHITE_STRAIGHT,
        BOUNDARY_CONTACT_YELLOW_CORNER,
        BOUNDARY_CONTACT_YELLOW_STRAIGHT,
        BOUNDARY_CORNER_KAPPA_THRESH,
        MAX_LAT_ERR_DEFAULT,
        MAX_LAT_ERR_INTERSECTION,
        OFF_ROAD_SHOULDER_DIST_Y,
        OFF_ROAD_SHOULDER_LAT_ERR,
        compute_step_road_state,
        in_any_corner,
        in_bend_turn,
        in_branch_permeable_gate,
        in_c1_turn,
        in_c2_turn,
        in_c4_turn,
        in_intersection_expert_zone,
        in_multi_branch_intersection,
        in_off_cliff_east,
        in_stop_bar,
        in_wrong_lane_intersection,
    )


def white_line_contact(
    env: ManagerBasedRLEnv,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
    threshold: float = 0.05,
) -> torch.Tensor:
    """
    Terminates if the robot hits a white or yellow line boundary, drives into the
    oncoming lane, or departs the drivable road envelope.
    Uses the canonical per-step road state cache (`compute_step_road_state`) so
    all spatial telemetry (including `go_signal` and `turn_token`) is fresh before RewardManager runs.
    """
    state = compute_step_road_state(env, asset_name=asset_cfg.name)

    x = state["x"]
    y = state["y"]
    lat_err = state["lat_err"]
    head_err = state["head_err"]
    kappa = state["kappa"]
    dist_y = state["dist_y"]
    dist_w = state["dist_w"]
    dist_g = state["dist_g"]
    lookahead_kappa = state["lookahead_kappa"]
    turn_token = state["turn_token"]
    branch_token = state.get("branch_token", turn_token)
    go_signal = state["go_signal"]
    multi_branch_active = bool(env.extras.get("multi_branch_active", False))

    # Ensure all precomputed signals are published in env.extras
    env.extras["lat_err"] = lat_err
    env.extras["head_err"] = head_err
    env.extras["kappa"] = kappa
    env.extras["dist_y"] = dist_y
    env.extras["dist_w"] = dist_w
    env.extras["dist_g"] = dist_g
    env.extras["lookahead_kappa"] = lookahead_kappa
    env.extras["turn_token"] = turn_token
    env.extras["branch_token"] = branch_token
    env.extras["go_signal"] = go_signal

    # 1. Curvature-Adaptive Physical Contact Thresholds:
    effective_kappa = torch.maximum(
        torch.abs(kappa),
        torch.abs(lookahead_kappa.view_as(kappa)),
    )
    corner_zone = (
        in_c1_turn(x, y)
        | in_c2_turn(x, y)
        | in_bend_turn(x, y)
        | in_c4_turn(x, y)
    )
    is_corner = (effective_kappa > BOUNDARY_CORNER_KAPPA_THRESH) | corner_zone

    dyn_threshold_w = torch.where(
        is_corner,
        torch.tensor(BOUNDARY_CONTACT_WHITE_CORNER, device=env.device),
        torch.tensor(BOUNDARY_CONTACT_WHITE_STRAIGHT, device=env.device),
    )
    dyn_threshold_y = torch.where(
        is_corner,
        torch.tensor(BOUNDARY_CONTACT_YELLOW_CORNER, device=env.device),
        torch.tensor(BOUNDARY_CONTACT_YELLOW_STRAIGHT, device=env.device),
    )

    # 2. Gate & Intersection permeability check:
    # Yellow median line is NEVER permeable anywhere on the circuit!
    # Permeability is granted to the transverse painted white stop bar and active multi-branch white exits.
    permeable_white_mask = in_branch_permeable_gate(
        x, y, dist_g, turn_token=branch_token, multi_branch_active=multi_branch_active
    )
    hit_white = (~permeable_white_mask) & (dist_w < dyn_threshold_w)
    hit_yellow = dist_y < dyn_threshold_y

    off_cliff_east = in_off_cliff_east(
        x, y, turn_token=branch_token, multi_branch_active=multi_branch_active
    )
    off_road_shoulder = (
        (~is_corner)
        & (~permeable_white_mask)
        & (lat_err < OFF_ROAD_SHOULDER_LAT_ERR)
        & (dist_y > OFF_ROAD_SHOULDER_DIST_Y)
    )

    # Open intersection envelope: allow up to 0.70m lateral drift where road widens,
    # while strictly enforcing physical boundary contacts (hit_white | hit_yellow).
    inter_box = (
        in_c2_turn(x, y)
        | in_intersection_expert_zone(x, y)
        | (in_multi_branch_intersection(x, y) if multi_branch_active else torch.zeros_like(is_corner))
    )
    max_lat_err = torch.where(
        inter_box,
        torch.tensor(MAX_LAT_ERR_INTERSECTION, device=env.device),
        torch.tensor(MAX_LAT_ERR_DEFAULT, device=env.device),
    )

    # Wrong-way / Oncoming lane termination across intersection exit and North road
    wrong_lane = in_wrong_lane_intersection(
        x, y, turn_token=branch_token, multi_branch_active=multi_branch_active
    )

    boundary_hit = (
        hit_white
        | hit_yellow
        | off_cliff_east
        | off_road_shoulder
        | wrong_lane
        | (torch.abs(lat_err) > max_lat_err)
    )

    # Check if we just started (less than 20 steps) to prevent spawn resets
    is_spawn = env.episode_length_buf < 20
    return (~is_spawn) & boundary_hit


def height_termination(
    env: ManagerBasedRLEnv,
    threshold: float = -0.05,
    max_height: float = 3.50,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
) -> torch.Tensor:
    """
    Terminates if the robot falls below the track surface (local Z < threshold) or flips/flies (local Z > max_height).
    Accommodates 3D elevation tracks up to Z = 2.5m with headroom.
    """
    asset = env.scene[asset_cfg.name]
    env_origins = env.scene.env_origins
    local_z = asset.data.root_pos_w[:, 2] - env_origins[:, 2]
    is_spawn = env.episode_length_buf < 20
    fallen = local_z < threshold
    flying = local_z > max_height
    return (~is_spawn) & (fallen | flying)


def stagnation_termination(
    env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")
) -> torch.Tensor:
    """Terminates if the robot is crawling or stuck, measured by waypoint progress.

    Never terminates a robot legally holding at a RED light (`go_signal <= 0.5`
    or inside `in_stop_bar` with red signal), and resets stagnation timers cleanly
    both while holding at a red signal and upon episode resets.
    """
    past_grace = env.episode_length_buf > 100

    cum_wp = env.extras.get("cumulative_wp_index", None)
    if cum_wp is None:
        return torch.zeros(env.num_envs, dtype=torch.bool, device=env.device)

    if (
        "stagnation_term_snapshot" not in env.extras
        or env.extras["stagnation_term_snapshot"].shape != cum_wp.shape
    ):
        env.extras["stagnation_term_snapshot"] = cum_wp.clone()
        env.extras["stagnation_term_timer"] = torch.zeros(
            env.num_envs, dtype=torch.long, device=env.device
        )

    go_signal = env.extras.get(
        "go_signal", torch.ones(env.num_envs, device=env.device, dtype=torch.float32)
    )
    is_go = go_signal > 0.5

    local_pos = env.extras.get("local_pos", None)
    dist_g = env.extras.get("dist_g", None)
    if local_pos is not None and dist_g is not None:
        stop_bar_mask = in_stop_bar(local_pos[:, 0], local_pos[:, 1], dist_g)
    else:
        stop_bar_mask = torch.zeros_like(is_go, dtype=torch.bool)

    holding_at_red = (~is_go) | (stop_bar_mask & (~is_go))

    timer = env.extras["stagnation_term_timer"]
    snapshot = env.extras["stagnation_term_snapshot"]

    # Freeze/reset timer and snapshot while legally holding at a red light so
    # departing when the light turns green starts with a full 100-step window.
    timer = torch.where(holding_at_red, torch.zeros_like(timer), timer + 1)
    snapshot = torch.where(holding_at_red, cum_wp.clone(), snapshot)

    # Every 100 steps (2 seconds), it MUST have progressed at least 25 waypoints (~0.25m, 0.13 m/s)
    check_now = (timer >= 100) & (~holding_at_red)
    progress = cum_wp - snapshot

    stagnated = (progress < 25.0) & check_now & past_grace & is_go & (~holding_at_red)

    # Reset snapshot and timer for envs that were checked
    env.extras["stagnation_term_snapshot"] = torch.where(
        check_now, cum_wp.clone(), snapshot
    )
    env.extras["stagnation_term_timer"] = torch.where(
        check_now, torch.zeros_like(timer), timer
    )

    # Also reset on episode reset
    if hasattr(env, "episode_length_buf"):
        just_reset = env.episode_length_buf <= 1
        env.extras["stagnation_term_snapshot"] = torch.where(
            just_reset,
            torch.zeros_like(cum_wp),
            env.extras["stagnation_term_snapshot"],
        )
        env.extras["stagnation_term_timer"] = torch.where(
            just_reset,
            torch.zeros_like(timer),
            env.extras["stagnation_term_timer"],
        )

    return stagnated


def fov_visibility_termination(
    env: ManagerBasedRLEnv,
    horizontal_aperture: float,
    focal_length: float,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
) -> torch.Tensor:
    """Terminates if the track centerline heading error exceeds half the camera FOV (driving blind)."""
    head_err = env.extras.get("head_err", torch.zeros(env.num_envs, device=env.device))
    fov_rad = 2.0 * math.atan(horizontal_aperture / (2.0 * focal_length))
    return torch.abs(head_err) > (fov_rad / 2.0)


def ground_contact_termination(
    env: ManagerBasedRLEnv, sensor_cfg: SceneEntityCfg, threshold: float = 1.0
) -> torch.Tensor:
    """Terminates if the robot (any part) physically touches the ground plane."""
    contact_sensor = env.scene[sensor_cfg.name]
    net_contact_forces = contact_sensor.data.net_forces_w_history
    force_magnitudes = torch.norm(net_contact_forces, dim=-1)
    max_body_force = torch.max(force_magnitudes, dim=-1)[0]
    max_force = torch.max(max_body_force, dim=-1)[0]

    is_spawn = env.episode_length_buf < 20
    return (~is_spawn) & (max_force > threshold)


import sys as _sys  # noqa: E402

_sys.modules.setdefault("mdp.terminations", _sys.modules[__name__])
_sys.modules.setdefault("arcproLab.mdp.terminations", _sys.modules[__name__])

