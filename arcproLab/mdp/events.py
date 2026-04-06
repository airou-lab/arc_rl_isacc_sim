# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import torch
from isaaclab.managers import SceneEntityCfg
from isaaclab.envs import ManagerBasedRLEnv

from mdp.track_manager import get_track_manager
import omni.physx
import omni.physx.scripts.utils as physx_utils

def reset_robot_to_lane(env: ManagerBasedRLEnv, env_ids: torch.Tensor, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")):
    """
    Event to reset the robot to a random waypoint on the track, snapped to the road surface.
    """
    asset = env.scene[asset_cfg.name]
    tm = get_track_manager(device=env.device)
    
    # Initialize tensors
    final_pos = torch.zeros((len(env_ids), 3), device=env.device)
    quats = torch.zeros((len(env_ids), 4), device=env.device)
    quats[:, 0] = 1.0 # Default identity
    
    # Select random waypoints for fallback
    num_wp = len(tm.waypoints)
    rand_indices = torch.randint(0, num_wp, (len(env_ids),), device=env.device)
    fallback_wps = tm.waypoints[rand_indices]
    
    # Raycast interface
    query = omni.physx.get_physx_scene_query_interface()
    
    for i, env_id in enumerate(env_ids):
        found_road = False
        max_retries = 50 # Even more retries
        
        for attempt in range(max_retries):
            wp_idx = torch.randint(0, len(tm.waypoints), (1,), device=env.device).item()
            wp = tm.waypoints[wp_idx]
            x, y = float(wp[0]), float(wp[1])
            
            # Use direct query
            hit = query.raycast_closest((x, y, 100.0), (0.0, 0.0, -1.0), 200.0)
            
            if hit["hit"]:
                # PhysX 5.1 standard uses camelCase for these keys in script
                hit_path = str(hit.get("rigidBody") or hit.get("collisionPath") or "")
                # DEBUG: Always print what we hit to find the keyword
                if i == 0: print(f"  [Raycast] Hit: {hit_path}")
                
                # Extremely broad check: just find ANYTHING that isn't the robot itself
                is_road = any(k in hit_path.lower() for k in ["road", "piece", "pavement", "drivable", "grass", "terrain", "tile"])
                
                if is_road:
                    final_pos[i, 0] = hit["position"][0]
                    final_pos[i, 1] = hit["position"][1]
                    final_pos[i, 2] = hit["position"][2] + 1.0 # Land from 1m
                    
                    # Update quat for this waypoint
                    import math
                    yaw = float(wp[2])
                    half_yaw = yaw / 2.0
                    quats[i, 0] = math.cos(half_yaw)
                    quats[i, 3] = math.sin(half_yaw)
                    
                    if i == 0: print(f"[Event] Reset Env 0: Snapped to road at Z={final_pos[i, 2]:.2f} (Path: {hit_path})")
                    found_road = True
                    break
        
        if not found_road:
            final_pos[i, 0] = float(fallback_wps[i, 0])
            final_pos[i, 1] = float(fallback_wps[i, 1])
            final_pos[i, 2] = 10.0 # Standard fallback
            if i == 0: print(f"[Event] Reset Env 0: [!] FAILED to find road mesh, using fallback Z=10.0")
    
    # Teleport
    asset.write_root_pose_to_sim(torch.cat([final_pos, quats], dim=-1), env_ids=env_ids)
    asset.write_root_velocity_to_sim(torch.zeros((len(env_ids), 6), device=env.device), env_ids=env_ids)
