# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import torch
from isaaclab.managers import SceneEntityCfg
from isaaclab.envs import ManagerBasedRLEnv

def reset_robot_to_fixed_spawn(env: ManagerBasedRLEnv, env_ids: torch.Tensor, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")):
    """
    Event to reset the robot to a FIXED starting waypoint.
    Removed raycast to ensure a stable, horizontal spawn on the flat track.
    """
    asset = env.scene[asset_cfg.name]
    
    # Target Waypoint: Nudged towards Yellow Line
    # Base: -16.25375 | Nudge: -16.15
    local_spawn_x, local_spawn_y = -16.15, 5.56
    spawn_yaw = -1.5708 # -90 degrees (Face South)
    
    # Get environment origins
    env_origins = env.scene.env_origins[env_ids]
    
    # Initialize tensors
    final_pos = torch.zeros((len(env_ids), 3), device=env.device)
    final_pos[:, 0] = env_origins[:, 0] + local_spawn_x
    final_pos[:, 1] = env_origins[:, 1] + local_spawn_y
    final_pos[:, 2] = env_origins[:, 2] + 0.05 # Stable horizontal height
    
    quats = torch.zeros((len(env_ids), 4), device=env.device)
    import math
    half_yaw = spawn_yaw / 2.0
    quats[:, 0] = math.cos(half_yaw)
    quats[:, 3] = math.sin(half_yaw)
    
    # Teleport
    asset.write_root_pose_to_sim(torch.cat([final_pos, quats], dim=-1), env_ids=env_ids)
    asset.write_root_velocity_to_sim(torch.zeros((len(env_ids), 6), device=env.device), env_ids=env_ids)
    
    # Zero joint velocities to prevent wheels from spinning on spawn
    joint_pos = asset.data.default_joint_pos[env_ids]
    joint_vel = torch.zeros_like(asset.data.joint_vel[env_ids])
    asset.write_joint_state_to_sim(joint_pos, joint_vel, env_ids=env_ids)
