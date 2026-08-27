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

def reset_robot_to_fixed_spawn(env: ManagerBasedRLEnv, env_ids: torch.Tensor, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")):
    if env_ids is None:
        env_ids = torch.arange(env.scene.num_envs, device=env.device)
    """
    Event to reset the robot to the starting waypoint with Domain Randomization.
    Randomizes X-offset, Heading, and Initial Velocity to force 'Recovery' learning.
    """
    asset = env.scene[asset_cfg.name]
    num_resets = len(env_ids)
    
    # Exactly on the path center (X=-16.197) at safe drop height
    base_spawn_x, base_spawn_y = -16.197, 5.50
    base_spawn_yaw = -1.5708 # Face South (Correct track direction)
    
    # 1. Domain Randomization (Hardening)
    # Disabled for debugging pure straight-line traversal
    rand_offset_x = torch.zeros(num_resets, device=env.device)
    rand_yaw = torch.zeros(num_resets, device=env.device)
    # Randomize Initial Velocity: Disabled for stability during drop
    rand_vel_x = torch.zeros(num_resets, device=env.device)
    
    # Get environment origins
    env_origins = env.scene.env_origins[env_ids]
    
    # 2. Compute final positions
    final_pos = torch.zeros((num_resets, 3), device=env.device)
    final_pos[:, 0] = env_origins[:, 0] + base_spawn_x + rand_offset_x
    final_pos[:, 1] = env_origins[:, 1] + base_spawn_y
    final_pos[:, 2] = env_origins[:, 2] + 0.12 # Safe drop height

    # 3. Compute final rotations (WXYZ)
    quats = torch.zeros((num_resets, 4), device=env.device)
    
    for i, env_id in enumerate(env_ids):
        # Standard upright Yaw rotation (Z-axis)
        yaw_offset = rand_yaw[i].item()
        half_yaw = (base_spawn_yaw + yaw_offset) / 2.0
        
        quats[i, 0] = math.cos(half_yaw)
        quats[i, 1] = 0.0
        quats[i, 2] = 0.0
        quats[i, 3] = math.sin(half_yaw)
    # 4. Apply Initial Velocity (World Frame)
    velocities = torch.zeros((num_resets, 6), device=env.device)
    
    # Map local forward speed (rand_vel_x) to world frame based on yaw
    # Vx = V * cos(yaw), Vy = V * sin(yaw)
    velocities[:, 0] = rand_vel_x * torch.cos(base_spawn_yaw + rand_yaw)
    velocities[:, 1] = rand_vel_x * torch.sin(base_spawn_yaw + rand_yaw)

    # Teleport and Apply Initial State
    asset.write_root_pose_to_sim(torch.cat([final_pos, quats], dim=-1), env_ids=env_ids)
    asset.write_root_velocity_to_sim(velocities, env_ids=env_ids)
    
    # Zero joint states
    joint_pos = asset.data.default_joint_pos[env_ids]
    joint_vel = torch.zeros_like(asset.data.joint_vel[env_ids])
    asset.write_joint_state_to_sim(joint_pos, joint_vel, env_ids=env_ids)
    # Reset the cumulative distance obs accumulator for these envs (F2).
    # observations.py slot 11 (`distance`) integrates root_lin_vel_b[:,0]*0.05
    # every step and was previously never zeroed across resets — it drifted
    # unbounded and VecNormalize's running stats never stabilized. Zero it
    # only for the env_ids being reset (preserves accumulators for others
    # in the same vec batch).
    if "distance" in env.extras:
        env.extras["distance"][env_ids] = 0.0

    if "prev_action" in env.extras:
        env.extras["prev_action"][env_ids] = 0.0

    # Reset the go_signal FSM for these envs (T3.2). On reset the robot is
    # placed behind the same stop bar it had been past; without a reset,
    # the FSM would still be in DEPART or STOP state from the prior episode
    # and the policy would inherit a stale go_signal.
    try:
        from mdp.go_signal_manager import get_go_signal_manager
        mgr = get_go_signal_manager(num_envs=env.num_envs, device=env.device)
        mgr.reset(env_ids)
    except Exception:
        pass
