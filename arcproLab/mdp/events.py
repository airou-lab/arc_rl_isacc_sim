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
    """
    Event to reset the robot to the starting waypoint with Domain Randomization.
    Randomizes X-offset, Heading, and Initial Velocity to force 'Recovery' learning.
    """
    asset = env.scene[asset_cfg.name]
    num_resets = len(env_ids)
    
    # Base Target: Lane Center (Sync with -16.2 centerline)
    base_spawn_x, base_spawn_y = -16.18, 5.30
    base_spawn_yaw = 1.5708 # Face North
    
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
    final_pos[:, 2] = env_origins[:, 2] + 0.20 # Safe drop height

    # 3. Compute final rotations (WXYZ)
    quats = torch.zeros((num_resets, 4), device=env.device)
    
    # Ground Alignment (Raycast for Normal)
    query = omni.physx.get_physx_scene_query_interface()
    for i, env_id in enumerate(env_ids):
        world_x, world_y = final_pos[i, 0].item(), final_pos[i, 1].item()
        hit = query.raycast_closest((world_x, world_y, 100.0), (0.0, 0.0, -1.0), 200.0)
        
        # 180-degree Roll (X-axis) * Yaw (Z-axis)
        # Combine: q_final = q_yaw * q_roll
        # Base North+Upright: w=0, x=0.7071, y=0.7071, z=0
        
        yaw_offset = rand_yaw[i].item()
        half_yaw = (base_spawn_yaw + yaw_offset) / 2.0
        c = math.cos(half_yaw)
        s = math.sin(half_yaw)
        
        # Combined WXYZ result:
        quats[i, 0] = 0.0 * c - 1.0 * 0.0 # W
        quats[i, 1] = 0.7071 * c + 0.7071 * s # X
        quats[i, 2] = 0.7071 * c - 0.7071 * s # Y
        quats[i, 3] = 0.0 * c + 0.0 * s # Z
        
        # Standard upright Yaw rotation (Z-axis)
        # W = cos(yaw/2)
        # X = 0
        # Y = 0
        # Z = sin(yaw/2)
        quats[i, 0] = math.cos(half_yaw)
        quats[i, 1] = 0.0
        quats[i, 2] = 0.0
        quats[i, 3] = math.sin(half_yaw)

        if hit["hit"]:
            # Snap Z to road (20cm offset)
            spawn_z = hit["position"][2] + 0.20
            final_pos[i, 2] = spawn_z
    
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
