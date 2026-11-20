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
    from mdp.actions import get_normalized_actions
    from mdp.curriculum_config import get_lookahead_distance
    from mdp.track_manager import get_track_manager
    from mdp.track_topology import (
        TELEMETRY_DIM,
        TELEMETRY_NAMES,
        compute_step_road_state,
    )
except ImportError:
    from arcproLab.mdp.actions import get_normalized_actions
    from arcproLab.mdp.curriculum_config import get_lookahead_distance
    from arcproLab.mdp.track_manager import get_track_manager
    from arcproLab.mdp.track_topology import (
        TELEMETRY_DIM,
        TELEMETRY_NAMES,
        compute_step_road_state,
    )


def get_telemetry_vector(
    env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")
) -> torch.Tensor:
    return _compute_telemetry(env, asset_cfg, masked=True)


def get_critic_state_vector(
    env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")
) -> torch.Tensor:
    return _compute_telemetry(env, asset_cfg, masked=False)


def _compute_telemetry(
    env: ManagerBasedRLEnv,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
    masked: bool = True,
) -> torch.Tensor:
    """
    Constructs the canonical 12-element telemetry vector (`TELEMETRY_NAMES`) for the ARCPro policy:
      0: turn_token
      1: go_signal
      2: goal_dist
      3: speed
      4: yaw_rate
      5: prev_steer
      6: prev_drive
      7: lat_vel_b
      8: lat_err
      9: head_err
     10: lookahead_kappa
     11: distance

    Uses `compute_step_road_state(env)` to avoid redundant 396.5k-point `torch.cdist` calls
    and ensure zero GPU-CPU `.item()` synchronization stalls during stepping.
    """
    state = compute_step_road_state(env, asset_name=asset_cfg.name)
    asset = env.scene[asset_cfg.name]

    obs = torch.zeros((env.num_envs, TELEMETRY_DIM), device=env.device, dtype=torch.float32)

    # Slot 0: turn_token
    obs[:, 0] = state["turn_token"]

    # Slot 1: go_signal (cached in env.extras["go_signal"])
    go_signal = state["go_signal"]
    env.extras["go_signal"] = go_signal
    obs[:, 1] = go_signal

    # Slot 2: goal_dist
    obs[:, 2] = state["goal_dist"]

    # Slot 3: Forward Speed (m/s)
    obs[:, 3] = torch.norm(asset.data.root_lin_vel_b[:, :2], dim=1)

    # Slot 4: Yaw Rate (rad/s) - Local Z angular velocity
    obs[:, 4] = asset.data.root_ang_vel_b[:, 2]

    # Slots 5-6: Normalized Last Actions (prev_steer, prev_drive) strictly in [-1.0, 1.0]
    obs[:, 5:7] = get_normalized_actions(env)

    # Slot 7: Lateral Velocity (m/s) in body frame
    obs[:, 7] = torch.clamp(asset.data.root_lin_vel_b[:, 1], min=-2.0, max=2.0)

    # Track progress logging & distance accumulation (updated once per physics step)
    tm = get_track_manager(device=env.device)
    if (
        "distance" not in env.extras
        or not isinstance(env.extras["distance"], torch.Tensor)
        or env.extras["distance"].shape[0] != env.num_envs
    ):
        env.extras["distance"] = torch.zeros(
            env.num_envs, device=env.device, dtype=torch.float32
        )

    if masked and tm.last_indices is not None and tm.waypoints is not None:
        num_wps = tm.waypoints.shape[0]
        delta_wp = env.extras.get(
            "cumulative_wp_index", torch.zeros_like(tm.last_indices)
        ).long()
        track_progress_pct = (delta_wp.float() / max(num_wps, 1)) * 100.0
        env.extras["track_progress_pct"] = track_progress_pct
        env.extras["track_wp_delta"] = delta_wp

    reset_buf = getattr(env, "reset_buf", None)
    if reset_buf is not None and masked:
        env.extras["distance"] = torch.where(
            reset_buf,
            torch.zeros_like(env.extras["distance"]),
            env.extras["distance"],
        )
        raw_head_err = state["raw_head_err"]
        new_dir = torch.where(torch.abs(raw_head_err) > 1.5708, -1.0, 1.0).to(
            device=env.device, dtype=torch.float32
        )
        env.extras["track_dir"] = torch.where(
            reset_buf, new_dir, env.extras["track_dir"]
        )

    if masked:
        step_dt = float(getattr(env, "step_dt", 0.02))
        # F1Tenth root +X points toward the rear wing; negate to accumulate forward distance
        step_dist = -asset.data.root_lin_vel_b[:, 0] * step_dt
        step_dist = torch.nan_to_num(step_dist, nan=0.0, posinf=0.0, neginf=0.0)
        if reset_buf is not None:
            step_dist = torch.where(
                reset_buf, torch.zeros_like(step_dist), step_dist
            )
        env.extras["distance"] = torch.nan_to_num(
            env.extras["distance"] + step_dist, nan=0.0, posinf=0.0, neginf=0.0
        )

    # Slots 8-11: Privileged & Curvature Telemetry
    obs[:, 8] = state["lat_err"]
    obs[:, 9] = state["head_err"]
    obs[:, 10] = state["lookahead_kappa"]
    obs[:, 11] = env.extras["distance"]

    # Guard against NaNs or Infs
    obs = torch.nan_to_num(obs, nan=0.0, posinf=0.0, neginf=0.0)
    return obs


import sys as _sys  # noqa: E402

_sys.modules.setdefault("mdp.observations", _sys.modules[__name__])
_sys.modules.setdefault("arcproLab.mdp.observations", _sys.modules[__name__])

