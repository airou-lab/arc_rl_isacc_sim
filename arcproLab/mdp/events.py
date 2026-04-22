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

def reset_robot_to_fixed_spawn(env: ManagerBasedRLEnv, env_ids: torch.Tensor, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")):
    """
    Event to reset the robot to a FIXED starting waypoint, aligned to the ground normal.
    """
    asset = env.scene[asset_cfg.name]
    
    # Target Waypoint: Lane Center
    # Moved North to Y=5.65 to ensure front wheels are on the road mesh
    local_spawn_x, local_spawn_y = -16.2616, 5.65
    spawn_yaw = -1.5708 # Face South
    
    # Get environment origins
    env_origins = env.scene.env_origins[env_ids]
    
    # 1. Base position
    final_pos = torch.zeros((len(env_ids), 3), device=env.device)
    final_pos[:, 0] = env_origins[:, 0] + local_spawn_x
    final_pos[:, 1] = env_origins[:, 1] + local_spawn_y
    final_pos[:, 2] = env_origins[:, 2] + 0.14 # Clean drop height

    # 2. Base rotation (Facing South)
    quats = torch.zeros((len(env_ids), 4), device=env.device)
    import math
    
    # 3. Ground Alignment (Raycast for Normal)
    query = omni.physx.get_physx_scene_query_interface()
    for i, env_id in enumerate(env_ids):
        world_x, world_y = final_pos[i, 0].item(), final_pos[i, 1].item()
        hit = query.raycast_closest((world_x, world_y, 100.0), (0.0, 0.0, -1.0), 200.0)
        
        # Default flat South-facing
        half_yaw = spawn_yaw / 2.0
        q_final = Gf.Quatd(math.cos(half_yaw), Gf.Vec3d(0, 0, math.sin(half_yaw)))

        if hit["hit"]:
            # Snap Z to road
            final_pos[i, 2] = hit["position"][2] + 0.08
            
            # Align orientation to ground normal
            normal = hit["normal"]
            normal_vec = Gf.Vec3d(normal[0], normal[1], normal[2])
            
            # Rotation from world-up (0,0,1) to ground-normal
            up_vec = Gf.Vec3d(0, 0, 1)
            tilt_rot = Gf.Rotation(up_vec, normal_vec)
            
            # Combine Yaw + Tilt
            yaw_rot = Gf.Rotation(Gf.Vec3d(0, 0, 1), math.degrees(spawn_yaw))
            q_final = (yaw_rot * tilt_rot).GetQuat()
            
        # Update tensor (WXYZ)
        quats[i, 0] = q_final.GetReal()
        quats[i, 1] = q_final.GetImaginary()[0]
        quats[i, 2] = q_final.GetImaginary()[1]
        quats[i, 3] = q_final.GetImaginary()[2]
    
    # Teleport and Zero Momentum
    asset.write_root_pose_to_sim(torch.cat([final_pos, quats], dim=-1), env_ids=env_ids)
    asset.write_root_velocity_to_sim(torch.zeros((len(env_ids), 6), device=env.device), env_ids=env_ids)
    
    # Zero joint states
    joint_pos = asset.data.default_joint_pos[env_ids]
    joint_vel = torch.zeros_like(asset.data.joint_vel[env_ids])
    asset.write_joint_state_to_sim(joint_pos, joint_vel, env_ids=env_ids)
