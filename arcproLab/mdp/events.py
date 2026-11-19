# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import math
import os

import torch

try:
    from isaaclab.envs import ManagerBasedRLEnv
    from isaaclab.managers import SceneEntityCfg
except ImportError:
    ManagerBasedRLEnv = object  # type: ignore[misc,assignment]

    class SceneEntityCfg:  # type: ignore[no-redef]
        def __init__(self, name: str = "robot"):
            self.name = name

try:
    from mdp.curriculum_config import get_sector_thresholds
    from mdp.track_topology import SECTOR_DEFS, invalidate_step_cache
except ImportError:
    from arcproLab.mdp.curriculum_config import get_sector_thresholds
    from arcproLab.mdp.track_topology import SECTOR_DEFS, invalidate_step_cache


def reset_robot_curriculum_spawn(
    env: ManagerBasedRLEnv,
    env_ids: torch.Tensor,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
):
    """
    5-Sector Certified Circuit Spawner using canonical `SECTOR_DEFS` from `track_topology.py`:
      - Sector 1: (-16.201,  5.500, -1.5708)     West Runway South Start
      - Sector 2: (-16.204, -3.500, -1.5708)     Corner 1 Runway Approach (~4m clean runway)
      - Sector 3: (-12.500, -7.100,  0.0000)     Leg 2 Clean Straightaway East
      - Sector 4: ( -7.000, -7.100,  0.0000)     Leg 2 Intersection Approach
      - Sector 5: (-10.500,  7.100,  3.14159265) Top North Straightaway West & C4 Approach
    Applies ±1.0cm lateral jitter and ±2° heading noise, resets all per-episode state trackers,
    recomputes `track_dir` for `env_ids`, and invalidates the per-step road state cache atomically.
    """
    if env_ids is None:
        env_ids = torch.arange(env.scene.num_envs, device=env.device)
    asset = env.scene[asset_cfg.name]
    num_resets = len(env_ids)

    env_origins = env.scene.env_origins[env_ids]
    final_pos = torch.zeros((num_resets, 3), device=env.device)
    quats = torch.zeros((num_resets, 4), device=env.device)

    forced_sector = os.environ.get("ARCPRO_FORCE_SECTOR")
    if forced_sector is not None and forced_sector.strip() in ("1", "2", "3", "4", "5"):
        s_idx = int(forced_sector.strip())
        cx, cy, cyaw = SECTOR_DEFS[s_idx]
        base_x = torch.full((num_resets,), cx, device=env.device)
        base_y = torch.full((num_resets,), cy, device=env.device)
        base_yaw = torch.full((num_resets,), cyaw, device=env.device)
    else:
        t1, t2, t3, t4 = get_sector_thresholds()
        r = torch.rand(num_resets, device=env.device)
        s1_x, s1_y, s1_yaw = SECTOR_DEFS[1]
        s2_x, s2_y, s2_yaw = SECTOR_DEFS[2]
        s3_x, s3_y, s3_yaw = SECTOR_DEFS[3]
        s4_x, s4_y, s4_yaw = SECTOR_DEFS[4]
        s5_x, s5_y, s5_yaw = SECTOR_DEFS[5]

        base_x = torch.where(
            r < t1,
            torch.tensor(s1_x, device=env.device),
            torch.where(
                r < t2,
                torch.tensor(s2_x, device=env.device),
                torch.where(
                    r < t3,
                    torch.tensor(s3_x, device=env.device),
                    torch.where(
                        r < t4,
                        torch.tensor(s4_x, device=env.device),
                        torch.tensor(s5_x, device=env.device),
                    ),
                ),
            ),
        )

        base_y = torch.where(
            r < t1,
            torch.tensor(s1_y, device=env.device),
            torch.where(
                r < t2,
                torch.tensor(s2_y, device=env.device),
                torch.where(
                    r < t3,
                    torch.tensor(s3_y, device=env.device),
                    torch.where(
                        r < t4,
                        torch.tensor(s4_y, device=env.device),
                        torch.tensor(s5_y, device=env.device),
                    ),
                ),
            ),
        )

        base_yaw = torch.where(
            r < t1,
            torch.tensor(s1_yaw, device=env.device),
            torch.where(
                r < t2,
                torch.tensor(s2_yaw, device=env.device),
                torch.where(
                    r < t3,
                    torch.tensor(s3_yaw, device=env.device),
                    torch.where(
                        r < t4,
                        torch.tensor(s4_yaw, device=env.device),
                        torch.tensor(s5_yaw, device=env.device),
                    ),
                ),
            ),
        )

    # Domain Randomization: Lateral Jitter (±1.0cm) & Heading Noise (±2 deg)
    lat_jitter = (torch.rand(num_resets, device=env.device) * 2.0 - 1.0) * 0.010
    head_noise = (torch.rand(num_resets, device=env.device) * 2.0 - 1.0) * (
        2.0 * math.pi / 180.0
    )

    spawn_x = base_x - torch.sin(base_yaw) * lat_jitter
    spawn_y = base_y + torch.cos(base_yaw) * lat_jitter
    spawn_yaw = base_yaw + head_noise

    final_pos[:, 0] = env_origins[:, 0] + spawn_x
    final_pos[:, 1] = env_origins[:, 1] + spawn_y
    final_pos[:, 2] = env_origins[:, 2] + 0.12  # Safe drop height

    half_yaw = spawn_yaw / 2.0
    quats[:, 0] = torch.cos(half_yaw)
    quats[:, 1] = 0.0
    quats[:, 2] = 0.0
    quats[:, 3] = torch.sin(half_yaw)

    velocities = torch.zeros((num_resets, 6), device=env.device)

    # Teleport and Apply Initial State
    asset.write_root_pose_to_sim(torch.cat([final_pos, quats], dim=-1), env_ids=env_ids)
    asset.write_root_velocity_to_sim(velocities, env_ids=env_ids)

    # Zero joint states
    joint_pos = asset.data.default_joint_pos[env_ids]
    joint_vel = torch.zeros_like(asset.data.joint_vel[env_ids])
    asset.write_joint_state_to_sim(joint_pos, joint_vel, env_ids=env_ids)

    if not hasattr(env, "extras") or env.extras is None:
        env.extras = {}

    if "distance" in env.extras:
        env.extras["distance"][env_ids] = 0.0

    if "prev_action_jerk" in env.extras:
        env.extras["prev_action_jerk"][env_ids] = 0.0
    if "prev_action_smoothness" in env.extras:
        env.extras["prev_action_smoothness"][env_ids] = 0.0

    # Reset waypoint progress & stagnation trackers atomically
    if "cumulative_wp_index" in env.extras:
        env.extras["cumulative_wp_index"][env_ids] = 0.0
    if "max_cumulative_wp_index" in env.extras:
        env.extras["max_cumulative_wp_index"][env_ids] = 0.0
    if "reward_wp_delta_step" in env.extras:
        env.extras["reward_wp_delta_step"][env_ids] = 0.0
    if "last_wp_check" in env.extras:
        env.extras["last_wp_check"][env_ids] = 0.0
    if "stagnation_term_snapshot" in env.extras:
        env.extras["stagnation_term_snapshot"][env_ids] = 0.0
    if "stagnation_term_timer" in env.extras:
        env.extras["stagnation_term_timer"][env_ids] = 0

    # Re-index TrackManager and recompute track_dir for env_ids
    try:
        try:
            from mdp.track_manager import get_track_manager
        except ImportError:
            from arcproLab.mdp.track_manager import get_track_manager

        tm = get_track_manager(device=env.device)
        spawn_xy = torch.stack([spawn_x, spawn_y], dim=-1)
        tm.reset_indices(env_ids=env_ids, new_pos=spawn_xy)
        if tm.last_indices is not None and tm.waypoints is not None and len(tm.waypoints) > 0:
            new_wp_idx = tm.last_indices[env_ids]
            if "prev_wp_idx_reward" in env.extras:
                env.extras["prev_wp_idx_reward"][env_ids] = new_wp_idx
            obs_yaw = spawn_yaw + math.pi
            obs_yaw = torch.atan2(torch.sin(obs_yaw), torch.cos(obs_yaw))
            wp_yaw = tm.waypoints[new_wp_idx, 2]
            raw_head_err = obs_yaw - wp_yaw
            raw_head_err = torch.atan2(torch.sin(raw_head_err), torch.cos(raw_head_err))
            new_track_dir = torch.where(
                torch.abs(raw_head_err) > 1.5708, -1.0, 1.0
            ).to(device=env.device, dtype=torch.float32)
            if (
                "track_dir" not in env.extras
                or not isinstance(env.extras["track_dir"], torch.Tensor)
                or env.extras["track_dir"].shape[0] != env.num_envs
            ):
                env.extras["track_dir"] = torch.ones(
                    env.num_envs, device=env.device, dtype=torch.float32
                )
            env.extras["track_dir"][env_ids] = new_track_dir
    except Exception:
        if "track_dir" in env.extras:
            env.extras["track_dir"][env_ids] = 1.0

    # Reset GoSignalManager
    try:
        try:
            from mdp.go_signal_manager import get_go_signal_manager
        except ImportError:
            from arcproLab.mdp.go_signal_manager import get_go_signal_manager

        mgr = get_go_signal_manager(num_envs=env.num_envs, device=env.device)
        mgr.reset(env_ids)
        if "go_signal" in env.extras:
            env.extras["go_signal"][env_ids] = 1.0
    except Exception:
        pass

    # Invalidate step road state cache atomically so ObservationManager computes fresh post-spawn telemetry
    invalidate_step_cache(env, env_ids=env_ids)


import sys as _sys  # noqa: E402

_sys.modules.setdefault("mdp.events", _sys.modules[__name__])
_sys.modules.setdefault("arcproLab.mdp.events", _sys.modules[__name__])

