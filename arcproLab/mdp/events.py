# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import torch
from isaaclab.managers import SceneEntityCfg
from isaaclab.envs import ManagerBasedRLEnv

from .track_manager import get_track_manager
import omni.physx
import omni.physx.scripts.utils as physx_utils

def reset_robot_to_lane(env: ManagerBasedRLEnv, env_ids: torch.Tensor, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")):
    """
    Event to reset the robot to a random waypoint on the track, snapped to the road surface.
    """
    asset = env.scene[asset_cfg.name]
    tm = get_track_manager(device=env.device, num_envs=env.num_envs)
    
    # Initialize tensors
    final_pos = torch.zeros((len(env_ids), 3), device=env.device)
    quats = torch.zeros((len(env_ids), 4), device=env.device)
    quats[:, 0] = 1.0 # Default identity
    
    # Select random waypoints for all environments
    num_wp = len(tm.waypoints)
    rand_indices = torch.randint(0, num_wp, (len(env_ids),), device=env.device)
    target_wps = tm.waypoints[rand_indices]
    
    for i, env_id in enumerate(env_ids):
        wp = target_wps[i]
        final_pos[i, 0] = float(wp[0])
        final_pos[i, 1] = float(wp[1])
        # Use 0.05m height to ensure we are above the track meshes (Z ~ 0)
        final_pos[i, 2] = 0.05
        
        # Calculate quaternion from yaw
        import math
        yaw = float(wp[2])
        half_yaw = yaw / 2.0
        quats[i, 0] = math.cos(half_yaw)
        quats[i, 3] = math.sin(half_yaw)
        
        if i == 0:
            print(f"[Event] Reset Env 0: Snapped to Waypoint at ({final_pos[i, 0]:.2f}, {final_pos[i, 1]:.2f}, Z=0.05)")
    
    # Teleport
    asset.write_root_pose_to_sim(torch.cat([final_pos, quats], dim=-1), env_ids=env_ids)
    asset.write_root_velocity_to_sim(torch.zeros((len(env_ids), 6), device=env.device), env_ids=env_ids)
