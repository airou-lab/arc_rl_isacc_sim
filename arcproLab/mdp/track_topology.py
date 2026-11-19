# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Canonical Single-Source-of-Truth Track Topology, Spatial Predicates, Constants & Step Cache.

CRITICAL: This module MUST maintain ZERO `isaaclab` or `omni` imports at the top level.
It relies solely on pure Python, NumPy, and PyTorch so CPU unit tests, checkpoint probes,
and PPO agents can import it without launching Isaac Sim.
"""

from __future__ import annotations

import importlib.util
import os
import sys
import types
from typing import Any, Dict, Tuple

import torch

# Ensure seamless dual-import aliasing (`mdp.track_topology` <-> `arcproLab.mdp.track_topology`)
_MDP_DIR = os.path.dirname(os.path.abspath(__file__))
_ARCPRO_DIR = os.path.dirname(_MDP_DIR)
if importlib.util.find_spec("isaaclab") is None and "arcproLab" not in sys.modules:
    _pkg = types.ModuleType("arcproLab")
    _pkg.__path__ = [_ARCPRO_DIR]
    sys.modules["arcproLab"] = _pkg
if "mdp" not in sys.modules:
    _mdp_pkg = types.ModuleType("mdp")
    _mdp_pkg.__path__ = [_MDP_DIR]
    sys.modules["mdp"] = _mdp_pkg
if "arcproLab.mdp" not in sys.modules:
    _arc_mdp_pkg = sys.modules["mdp"]
    sys.modules["arcproLab.mdp"] = _arc_mdp_pkg
    if "arcproLab" in sys.modules:
        setattr(sys.modules["arcproLab"], "mdp", _arc_mdp_pkg)
sys.modules["mdp.track_topology"] = sys.modules[__name__]
sys.modules["arcproLab.mdp.track_topology"] = sys.modules[__name__]
setattr(sys.modules["mdp"], "track_topology", sys.modules[__name__])

# =============================================================================
# 1. CANONICAL 5-SECTOR CURRICULUM SPAWN DEFINITIONS
# =============================================================================

SECTOR_DEFS: Dict[int, Tuple[float, float, float]] = {
    1: (-16.201, 5.500, -1.5708),     # West Runway South Start
    2: (-16.204, -3.500, -1.5708),    # Corner 1 Runway Approach (~4m clean runway)
    3: (-12.500, -7.100, 0.0000),     # Leg 2 Clean Straightaway East (NEVER -15.200 inside C1!)
    4: (-7.000, -7.100, 0.0000),      # Leg 2 Intersection Approach
    5: (-10.500, 7.100, 3.14159265),  # Top North Straightaway West & C4 Approach
}

DEFAULT_SECTOR_WEIGHTS: Tuple[float, float, float, float, float] = (
    0.25,
    0.25,
    0.20,
    0.15,
    0.15,
)

DEFAULT_LOOKAHEAD_DISTANCE: float = 1.2

SECTOR_DESCRIPTIONS: Dict[str, str] = {
    "1": "West Runway South Start (-16.201, 5.500)",
    "2": "Corner 1 Runway Approach (-16.204, -3.500)",
    "3": "Leg 2 Clean Straightaway East (-12.500, -7.100)",
    "4": "Leg 2 Intersection Approach (-7.000, -7.100)",
    "5": "Top North Straightaway West & C4 Approach (-10.500, 7.100)",
}

# =============================================================================
# 2. CANONICAL BOUNDARY & TELEMETRY CONSTANTS
# =============================================================================

# Symmetric boundary warning margin (m) from chassis center to yellow/white lines.
# Verified 0 yellow and 0 white violations across all 5,000 centerline waypoints.
BOUNDARY_WARN_MARGIN: float = 0.170
BOUNDARY_WARN_MARGIN_YELLOW: float = BOUNDARY_WARN_MARGIN
BOUNDARY_WARN_MARGIN_WHITE: float = BOUNDARY_WARN_MARGIN

# Curvature-adaptive physical contact termination thresholds (m)
BOUNDARY_CONTACT_WHITE_CORNER: float = 0.012
BOUNDARY_CONTACT_WHITE_STRAIGHT: float = 0.015
BOUNDARY_CONTACT_YELLOW_CORNER: float = 0.005
BOUNDARY_CONTACT_YELLOW_STRAIGHT: float = 0.008
BOUNDARY_CORNER_KAPPA_THRESH: float = 0.15

# Lateral drift & shoulder envelope constants (m)
MAX_LAT_ERR_INTERSECTION: float = 0.70
MAX_LAT_ERR_DEFAULT: float = 0.40
OFF_ROAD_SHOULDER_LAT_ERR: float = -0.25
OFF_ROAD_SHOULDER_DIST_Y: float = 0.38
FLARED_LANE_WIDTH_THRESH: float = 0.45

# Telemetry & Action Mirroring Contract (12-slot layout)
TELEMETRY_DIM: int = 12
TELEMETRY_NAMES: Tuple[str, ...] = (
    "turn_token",
    "go_signal",
    "goal_dist",
    "speed",
    "yaw_rate",
    "prev_steer",
    "prev_drive",
    "lat_vel_b",
    "lat_err",
    "head_err",
    "lookahead_kappa",
    "distance",
)
TELEMETRY_MIRROR_SIGNS: Tuple[float, ...] = (
    -1.0,  # 0: turn_token
    1.0,   # 1: go_signal
    1.0,   # 2: goal_dist
    1.0,   # 3: speed
    -1.0,  # 4: yaw_rate
    -1.0,  # 5: prev_steer
    1.0,   # 6: prev_drive
    -1.0,  # 7: lat_vel_b
    -1.0,  # 8: lat_err
    -1.0,  # 9: head_err
    -1.0,  # 10: lookahead_kappa
    1.0,   # 11: distance
)
ACTION_MIRROR_SIGNS: Tuple[float, float] = (-1.0, 1.0)
ACTOR_MASKED_TELEMETRY_INDICES: Tuple[int, ...] = (5, 6, 8, 9, 11)


# =============================================================================
# 3. CANONICAL SPATIAL PREDICATES
# =============================================================================


def _prepare_xy(x: Any, y: Any) -> Tuple[torch.Tensor, torch.Tensor]:
    """Coerce (x, y) inputs (tensors, numpy arrays, or scalars) to matching float32 tensors."""
    if isinstance(x, torch.Tensor):
        tx = x
    else:
        device = y.device if isinstance(y, torch.Tensor) else "cpu"
        tx = torch.as_tensor(x, dtype=torch.float32, device=device)

    if isinstance(y, torch.Tensor):
        ty = y.to(device=tx.device)
    else:
        ty = torch.as_tensor(y, dtype=torch.float32, device=tx.device)

    if ty.ndim != tx.ndim and ty.numel() == tx.numel():
        ty = ty.view_as(tx)
    return tx, ty


def _prepare_xyg(
    x: Any, y: Any, dist_g: Any
) -> Tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
    """Coerce (x, y, dist_g) inputs to matching float32 tensors."""
    tx, ty = _prepare_xy(x, y)
    if isinstance(dist_g, torch.Tensor):
        tg = dist_g.to(device=tx.device)
    else:
        tg = torch.as_tensor(dist_g, dtype=torch.float32, device=tx.device)

    if tg.ndim != tx.ndim and tg.numel() == tx.numel():
        tg = tg.view_as(tx)
    return tx, ty, tg


def in_c1_turn(x: Any, y: Any) -> torch.Tensor:
    """Corner 1 turn envelope (West Runway South -> Leg 2 East)."""
    tx, ty = _prepare_xy(x, y)
    return (tx <= -15.00) & (ty <= -6.15)


def in_c2_turn(x: Any, y: Any) -> torch.Tensor:
    """Corner 2 / Intersection turn envelope (Leg 2 East -> Northbound Straight)."""
    tx, ty = _prepare_xy(x, y)
    return (tx >= -6.35) & (tx <= -1.80) & (ty >= -7.50) & (ty <= -2.50)


# Alias for intersection box checks
in_intersection_box = in_c2_turn


def in_bend_turn(x: Any, y: Any) -> torch.Tensor:
    """Sweeping North-East Bend turn envelope (Northbound -> Top Straight West)."""
    tx, ty = _prepare_xy(x, y)
    return (tx >= -7.00) & (tx <= -2.00) & (ty >= 2.40) & (ty <= 7.20)


def in_c4_turn(x: Any, y: Any) -> torch.Tensor:
    """Corner 4 turn envelope (Top Straight West -> West Runway South)."""
    tx, ty = _prepare_xy(x, y)
    return (tx <= -14.80) & (ty >= 5.85)


def in_any_corner(x: Any, y: Any) -> torch.Tensor:
    """Union of all four circuit corner envelopes (C1, C2, Bend, C4)."""
    tx, ty = _prepare_xy(x, y)
    return (
        in_c1_turn(tx, ty)
        | in_c2_turn(tx, ty)
        | in_bend_turn(tx, ty)
        | in_c4_turn(tx, ty)
    )


def in_stop_bar(x: Any, y: Any, dist_g: Any) -> torch.Tensor:
    """Transverse painted white stop bar zone at Leg 2 intersection entry."""
    tx, ty, tg = _prepare_xyg(x, y, dist_g)
    return (tg < 0.60) & (tx >= -6.50) & (tx <= -4.50) & (ty >= -7.50) & (ty <= -6.50)


def in_wrong_lane_north(x: Any, y: Any) -> torch.Tensor:
    """Oncoming Southbound (West) lane along the North road (median at X = -2.56)."""
    tx, ty = _prepare_xy(x, y)
    return (ty >= -3.50) & (ty <= 2.20) & (tx >= -3.50) & (tx < -2.56)


def in_wrong_lane_apex(x: Any, y: Any) -> torch.Tensor:
    """Oncoming lane across the curving median apex exiting Corner 2 into North road."""
    tx, ty = _prepare_xy(x, y)
    return (
        (ty >= -5.20)
        & (ty < -3.50)
        & (tx >= -3.50)
        & (tx < (-2.56 + 0.35 * (ty + 3.50)))
    )


def in_wrong_lane_intersection(x: Any, y: Any) -> torch.Tensor:
    """Union of oncoming wrong-lane zones across intersection exit and North road."""
    tx, ty = _prepare_xy(x, y)
    return in_wrong_lane_north(tx, ty) | in_wrong_lane_apex(tx, ty)


def in_off_cliff_east(x: Any, y: Any) -> torch.Tensor:
    """East void cliff beyond the intersection outer boundary."""
    tx, ty = _prepare_xy(x, y)
    return (tx > -2.08) & (ty >= -7.50) & (ty <= -3.60)


def in_intersection_expert_zone(x: Any, y: Any) -> torch.Tensor:
    """Intersection teacher guidance envelope (stop-bar approach + C2 intersection)."""
    tx, ty = _prepare_xy(x, y)
    return (tx >= -6.50) & (tx <= -1.80) & (ty >= -7.50) & (ty <= -2.50)


# =============================================================================
# 4. SHARED PER-STEP ROAD STATE CACHE
# =============================================================================


def invalidate_step_cache(env: Any, env_ids: torch.Tensor | None = None) -> None:
    """Invalidate per-step road state cache and mark env_ids for track_dir re-evaluation."""
    if not hasattr(env, "extras") or env.extras is None:
        return
    env.extras["_road_state_cache_valid"] = False
    num_envs = getattr(env, "num_envs", None)
    device = getattr(env, "device", "cpu")
    if num_envs is not None:
        mask = env.extras.get("_track_dir_reset_mask", None)
        if mask is None or not isinstance(mask, torch.Tensor) or mask.shape[0] != num_envs:
            mask = torch.zeros(num_envs, dtype=torch.bool, device=device)
        if env_ids is None:
            mask[:] = True
        else:
            mask[env_ids] = True
        env.extras["_track_dir_reset_mask"] = mask


def compute_step_road_state(
    env: Any, asset_name: str = "robot"
) -> Dict[str, torch.Tensor]:
    """Compute and cache per-step road geometry, boundary distances, turn tokens, and go_signal.

    Eliminates 3x redundant 396.5k-point `torch.cdist` calls per RL step across
    TerminationManager, RewardManager, and ObservationManager (policy + critic),
    and updates `go_signal` deterministically BEFORE terminations and rewards run.
    """
    if not hasattr(env, "extras") or env.extras is None:
        env.extras = {}

    device = getattr(env, "device", "cpu")
    num_envs = getattr(env, "num_envs", 1)

    # Extract robot pose from scene if available
    local_pos = None
    yaw = None
    try:
        scene = getattr(env, "scene", None)
        if scene is not None:
            asset = scene[asset_name]
            env_origins = scene.env_origins
            local_pos = asset.data.root_pos_w - env_origins
            q = asset.data.root_quat_w
            yaw = torch.atan2(
                2.0 * (q[:, 0] * q[:, 3] + q[:, 1] * q[:, 2]),
                1.0 - 2.0 * (q[:, 2] ** 2 + q[:, 3] ** 2),
            )
            # F1Tenth root +X points toward the rear wing; add pi to align with visual nose
            yaw = yaw + 3.14159265
            yaw = torch.atan2(torch.sin(yaw), torch.cos(yaw))
    except Exception:
        local_pos = None
        yaw = None

    # Fallback for minimal mock environments without scene/robot pose
    if local_pos is None or yaw is None:
        local_pos = env.extras.get(
            "local_pos", torch.zeros((num_envs, 3), device=device, dtype=torch.float32)
        )
        yaw = env.extras.get(
            "yaw", torch.zeros(num_envs, device=device, dtype=torch.float32)
        )
        fallback_state = {
            "local_pos": local_pos,
            "x": local_pos[:, 0],
            "y": local_pos[:, 1],
            "z": local_pos[:, 2] if local_pos.shape[1] >= 3 else torch.zeros_like(local_pos[:, 0]),
            "yaw": yaw,
            "raw_head_err": env.extras.get(
                "raw_head_err", torch.zeros(num_envs, device=device, dtype=torch.float32)
            ),
            "lat_err": env.extras.get(
                "lat_err", torch.zeros(num_envs, device=device, dtype=torch.float32)
            ),
            "head_err": env.extras.get(
                "head_err", torch.zeros(num_envs, device=device, dtype=torch.float32)
            ),
            "kappa": env.extras.get(
                "kappa", torch.zeros(num_envs, device=device, dtype=torch.float32)
            ),
            "dist_y": env.extras.get(
                "dist_y", torch.full((num_envs,), 0.21, device=device, dtype=torch.float32)
            ),
            "dist_w": env.extras.get(
                "dist_w", torch.full((num_envs,), 0.21, device=device, dtype=torch.float32)
            ),
            "dist_g": env.extras.get(
                "dist_g", torch.full((num_envs,), 10.0, device=device, dtype=torch.float32)
            ),
            "lookahead_kappa": env.extras.get(
                "lookahead_kappa", torch.zeros(num_envs, device=device, dtype=torch.float32)
            ),
            "turn_token": env.extras.get(
                "turn_token", torch.zeros(num_envs, device=device, dtype=torch.float32)
            ),
            "go_signal": env.extras.get(
                "go_signal", torch.ones(num_envs, device=device, dtype=torch.float32)
            ),
            "goal_dist": env.extras.get(
                "goal_dist", torch.zeros(num_envs, device=device, dtype=torch.float32)
            ),
            "track_dir": env.extras.get(
                "track_dir", torch.ones(num_envs, device=device, dtype=torch.float32)
            ),
        }
        return fallback_state

    step_counter = getattr(env, "common_step_counter", None)
    cache_valid = bool(env.extras.get("_road_state_cache_valid", False))
    if cache_valid:
        cached_step = env.extras.get("_road_state_step", None)
        cached_pos = env.extras.get("_road_state_pos", None)
        cached_yaw = env.extras.get("_road_state_yaw", None)
        cached_dict = env.extras.get("_road_state_dict", None)
        if (
            cached_dict is not None
            and cached_step == step_counter
            and isinstance(cached_pos, torch.Tensor)
            and cached_pos.shape == local_pos.shape
            and torch.equal(cached_pos, local_pos)
            and isinstance(cached_yaw, torch.Tensor)
            and cached_yaw.shape == yaw.shape
            and torch.equal(cached_yaw, yaw)
        ):
            return cached_dict

    try:
        from mdp.curriculum_config import get_lookahead_distance
        from mdp.track_manager import get_track_manager
    except ImportError:
        from arcproLab.mdp.curriculum_config import get_lookahead_distance
        from arcproLab.mdp.track_manager import get_track_manager

    tm = get_track_manager(device=device)
    lat_err, raw_head_err, kappa = tm.compute_errors(local_pos, yaw)

    computed_dir = torch.where(
        torch.abs(raw_head_err) > 1.5708, -1.0, 1.0
    ).to(device=device, dtype=torch.float32)

    if (
        "track_dir" not in env.extras
        or not isinstance(env.extras["track_dir"], torch.Tensor)
        or env.extras["track_dir"].shape != computed_dir.shape
    ):
        env.extras["track_dir"] = computed_dir
    else:
        reset_mask = env.extras.get("_track_dir_reset_mask", None)
        if isinstance(reset_mask, torch.Tensor) and reset_mask.shape == computed_dir.shape:
            env.extras["track_dir"] = torch.where(
                reset_mask, computed_dir, env.extras["track_dir"]
            )
            env.extras["_track_dir_reset_mask"] = None

    is_reverse = env.extras["track_dir"] < 0
    head_err = torch.where(
        is_reverse,
        raw_head_err - 3.14159265 * torch.sign(raw_head_err),
        raw_head_err,
    )
    lat_err = torch.where(is_reverse, -lat_err, lat_err)

    dist_y, dist_w, dist_g = tm.compute_marker_distances(local_pos)

    lookahead_kappa = tm.get_lookahead_curvature(
        lookahead_distance=get_lookahead_distance(),
        track_dir=env.extras["track_dir"],
    ).to(device=device, dtype=torch.float32)

    # RoadManager turn_token & goal_dist update
    try:
        try:
            from mdp.road_manager import get_road_manager
        except ImportError:
            from arcproLab.mdp.road_manager import get_road_manager

        rm = get_road_manager(num_envs=num_envs, num_agents=1, device=device)
        rm.update(env)
        turn_tokens, _ = rm.get_nav_commands()
        turn_token = turn_tokens.squeeze(1)
    except Exception:
        turn_token = torch.where(
            in_any_corner(local_pos[:, 0], local_pos[:, 1]),
            torch.full((num_envs,), -1.0, device=device, dtype=torch.float32),
            torch.zeros(num_envs, device=device, dtype=torch.float32),
        )

    goal_dist = torch.zeros(num_envs, device=device, dtype=torch.float32)

    # GoSignalManager update (runs deterministically BEFORE terminations & rewards,
    # and at most once per physics step counter so post-reset cache refreshes do not double-step dwell)
    try:
        try:
            from mdp.go_signal_manager import get_go_signal_manager
        except ImportError:
            from arcproLab.mdp.go_signal_manager import get_go_signal_manager

        mgr = get_go_signal_manager(num_envs=num_envs, device=device)
        should_step_go = (
            step_counter is None
            or env.extras.get("_go_signal_updated_step", None) != step_counter
        )
        if should_step_go:
            camera = (
                env.scene.get("tiled_camera")
                if hasattr(env.scene, "get")
                else None
            )
            if camera is None:
                try:
                    camera = env.scene["tiled_camera"]
                except Exception:
                    camera = None
            images = None
            if (
                camera is not None
                and hasattr(camera, "data")
                and isinstance(getattr(camera.data, "output", None), dict)
                and "rgb" in camera.data.output
            ):
                images = camera.data.output["rgb"]
            go_signal = mgr.update(images).clone()
            if step_counter is not None:
                env.extras["_go_signal_updated_step"] = step_counter
        else:
            go_signal = mgr.go_signal.clone()
    except Exception:
        go_signal = env.extras.get(
            "go_signal", torch.ones(num_envs, device=device, dtype=torch.float32)
        )

    # Publish all canonical keys to env.extras
    env.extras["local_pos"] = local_pos
    env.extras["yaw"] = yaw
    env.extras["raw_head_err"] = raw_head_err
    env.extras["lat_err"] = lat_err
    env.extras["head_err"] = head_err
    env.extras["kappa"] = kappa
    env.extras["dist_y"] = dist_y
    env.extras["dist_w"] = dist_w
    env.extras["dist_g"] = dist_g
    env.extras["lookahead_kappa"] = lookahead_kappa
    env.extras["turn_token"] = turn_token
    env.extras["go_signal"] = go_signal
    env.extras["goal_dist"] = goal_dist

    state_dict: Dict[str, torch.Tensor] = {
        "local_pos": local_pos,
        "x": local_pos[:, 0],
        "y": local_pos[:, 1],
        "z": local_pos[:, 2] if local_pos.shape[1] >= 3 else torch.zeros_like(local_pos[:, 0]),
        "yaw": yaw,
        "raw_head_err": raw_head_err,
        "lat_err": lat_err,
        "head_err": head_err,
        "kappa": kappa,
        "dist_y": dist_y,
        "dist_w": dist_w,
        "dist_g": dist_g,
        "lookahead_kappa": lookahead_kappa,
        "turn_token": turn_token,
        "go_signal": go_signal,
        "goal_dist": goal_dist,
        "track_dir": env.extras["track_dir"],
    }

    env.extras["_road_state_pos"] = local_pos.clone()
    env.extras["_road_state_yaw"] = yaw.clone()
    env.extras["_road_state_step"] = step_counter
    env.extras["_road_state_cache_valid"] = True
    env.extras["_road_state_dict"] = state_dict

    return state_dict


__all__ = [
    "SECTOR_DEFS",
    "DEFAULT_SECTOR_WEIGHTS",
    "DEFAULT_LOOKAHEAD_DISTANCE",
    "SECTOR_DESCRIPTIONS",
    "BOUNDARY_WARN_MARGIN",
    "BOUNDARY_WARN_MARGIN_YELLOW",
    "BOUNDARY_WARN_MARGIN_WHITE",
    "BOUNDARY_CONTACT_WHITE_CORNER",
    "BOUNDARY_CONTACT_WHITE_STRAIGHT",
    "BOUNDARY_CONTACT_YELLOW_CORNER",
    "BOUNDARY_CONTACT_YELLOW_STRAIGHT",
    "BOUNDARY_CORNER_KAPPA_THRESH",
    "MAX_LAT_ERR_INTERSECTION",
    "MAX_LAT_ERR_DEFAULT",
    "OFF_ROAD_SHOULDER_LAT_ERR",
    "OFF_ROAD_SHOULDER_DIST_Y",
    "FLARED_LANE_WIDTH_THRESH",
    "TELEMETRY_DIM",
    "TELEMETRY_NAMES",
    "TELEMETRY_MIRROR_SIGNS",
    "ACTION_MIRROR_SIGNS",
    "ACTOR_MASKED_TELEMETRY_INDICES",
    "in_c1_turn",
    "in_c2_turn",
    "in_intersection_box",
    "in_bend_turn",
    "in_c4_turn",
    "in_any_corner",
    "in_stop_bar",
    "in_wrong_lane_north",
    "in_wrong_lane_apex",
    "in_wrong_lane_intersection",
    "in_off_cliff_east",
    "in_intersection_expert_zone",
    "compute_step_road_state",
    "invalidate_step_cache",
]
