# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import torch
from isaaclab.managers import SceneEntityCfg
from isaaclab.envs import ManagerBasedRLEnv

def white_line_contact(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    """
    Terminates if any wheel hits the boundaries. 
    Calibrated for 8x robot (3.6m wide) and 0.73m mathematical offset.
    """
    asset = env.scene[asset_cfg.name]
    
    # 1. Physical Crash (High-force impact)
    sensor = env.scene.sensors["contact_forces"]
    forces = torch.norm(sensor.data.net_forces_w, dim=-1)
    chassis_crash = torch.any(forces > 5000.0, dim=1)
    
    # 2. Precision Lane Contact
    from mdp.track_manager import get_track_manager
    tm = get_track_manager(device=env.device)
    
    q = asset.data.root_quat_w
    yaw = torch.atan2(2.0 * (q[:, 0] * q[:, 3] + q[:, 1] * q[:, 2]), 1.0 - 2.0 * (q[:, 2]**2 + q[:, 3]**2))
    
    # Get direct math error in meters
    lat_err, _ = tm.compute_errors(asset.data.root_pos_w, yaw)
    
    # LANE BOUNDARIES (8x Scale)
    # Robot width is 3.6m. Right lane is 4.5m wide.
    # To keep WHEELS inside the 4.5m lane:
    # Reset if center < 0.5m (crosses yellow) or > 4.0m (crosses white)
    marker_hit = (lat_err < 0.5) | (lat_err > 4.0)

    # Apply 50-step settling buffer for the physics to 'link' (8x scale is heavy)
    settled = env.episode_length_buf > 50

    # Debug logging (env 0)
    if settled[0].item() and marker_hit[0].item():
        print(f"[TERMINATION] Lane Departure! LatErr: {lat_err[0].item():.3f}m (Allowed -0.5m to 4.5m)")

    return settled & (chassis_crash | marker_hit)

def height_termination(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    """Terminates if the robot flips or falls."""
    asset = env.scene[asset_cfg.name]
    height = asset.data.root_pos_w[:, 2]
    settled = env.episode_length_buf > 5
    return settled & ((height < 0.1) | (height > 2.0))

def stagnation_termination(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    """Terminates if the robot hasn't made forward progress in 100 steps."""
    # Apply 50-step settling buffer for the physics to 'link' (8x scale is heavy)
    settled = env.episode_length_buf > 50
    
    # Initialize extras if not present
    if "last_dist" not in env.extras:
        env.extras["last_dist"] = torch.zeros(env.num_envs, device=env.device)
        env.extras["stagnant_steps"] = torch.zeros(env.num_envs, device=env.device)

    current_dist = env.extras.get("distance", torch.zeros(env.num_envs, device=env.device))
    
    # Check progress only if settled
    # (Reset counter if we haven't settled yet to avoid instant reset)
    progress = (current_dist - env.extras["last_dist"]) > 0.1
    env.extras["stagnant_steps"] = torch.where(~settled | progress, 
                                               torch.zeros_like(env.extras["stagnant_steps"]), 
                                               env.extras["stagnant_steps"] + 1)
    
    # Update last distance for next check
    env.extras["last_dist"] = current_dist.clone()
    
    # Terminate if stuck for more than 100 steps AFTER settling
    stuck = env.extras["stagnant_steps"] > 100
    
    if stuck[0].item():
        print(f"[TERMINATION] Robot Stagnant! No progress for 100 steps.")
        
    return stuck
