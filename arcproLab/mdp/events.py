# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import torch
import numpy as np
from isaaclab.managers import SceneEntityCfg
from isaaclab.envs import ManagerBasedRLEnv
import omni.physx
from pxr import UsdGeom, Usd, Gf
import math

def reset_robot_curriculum_spawn(env: ManagerBasedRLEnv, env_ids: torch.Tensor, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")):
    """
    Curriculum Spawner:
    50% fixed starting line, 50% uniformly sampled along the 5,000 track waypoints.
    Trains the agent across all 4 corners and straightaways simultaneously.
    """
    if env_ids is None:
        env_ids = torch.arange(env.scene.num_envs, device=env.device)
    asset = env.scene[asset_cfg.name]
    num_resets = len(env_ids)

    from mdp.track_manager import get_track_manager
    tm = get_track_manager(device=env.device)
    
    base_spawn_x, base_spawn_y = -16.197, 5.50
    base_spawn_yaw = -1.5708 # Face South

    env_origins = env.scene.env_origins[env_ids]
    final_pos = torch.zeros((num_resets, 3), device=env.device)
    quats = torch.zeros((num_resets, 4), device=env.device)

    # 100% Fixed Start Line Baseline (Clean 707-step continuous full-lap run)
    spawn_x = torch.full((num_resets,), base_spawn_x, device=env.device)
    spawn_y = torch.full((num_resets,), base_spawn_y, device=env.device)
    spawn_yaw = torch.full((num_resets,), base_spawn_yaw, device=env.device)

    final_pos[:, 0] = env_origins[:, 0] + spawn_x
    final_pos[:, 1] = env_origins[:, 1] + spawn_y
    final_pos[:, 2] = env_origins[:, 2] + 0.12 # Safe drop height

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

    if "distance" in env.extras:
        env.extras["distance"][env_ids] = 0.0

    if "prev_action_jerk" in env.extras:
        env.extras["prev_action_jerk"][env_ids] = 0.0
    if "prev_action_smoothness" in env.extras:
        env.extras["prev_action_smoothness"][env_ids] = 0.0

    try:
        from mdp.go_signal_manager import get_go_signal_manager
        mgr = get_go_signal_manager(num_envs=env.num_envs, device=env.device)
        mgr.reset(env_ids)
    except Exception:
        pass
