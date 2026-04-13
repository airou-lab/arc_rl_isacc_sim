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
    Calibrated for 1x robot (0.45m wide).
    """
    asset = env.scene[asset_cfg.name]
    
    # 1. Physical Crash (High-force impact)
    # Reverted threshold from 5000.0 (8x) to 100.0 (1x)
    sensor = env.scene.sensors["contact_forces"]
    forces = torch.norm(sensor.data.net_forces_w, dim=-1)
    chassis_crash = torch.any(forces > 100.0, dim=1)
    
    # 2. Precision Lane Contact
    from mdp.track_manager import get_track_manager
    tm = get_track_manager(device=env.device)
    
    q = asset.data.root_quat_w
    yaw = torch.atan2(2.0 * (q[:, 0] * q[:, 3] + q[:, 1] * q[:, 2]), 1.0 - 2.0 * (q[:, 2]**2 + q[:, 3]**2))
    
    # Get environment-relative position
    env_origins = env.scene.env_origins
    local_pos = asset.data.root_pos_w - env_origins
    
    # Get direct math error in meters using local position
    lat_err, _ = tm.compute_errors(local_pos, yaw)
    
    # LANE BOUNDARIES (1x Scale)
    # Robot width is 0.45m. Right lane is ~1.125m wide.
    # To keep WHEELS inside the ~1.125m lane:
    # center is allowed ~ +/- 0.3m? 
    # Let's use 0.5m as reset threshold for simplicity
    marker_hit = torch.abs(lat_err) > 0.5

    # Lower settling buffer for light 1x physics
    settled = env.episode_length_buf > 10

    # Debug logging (env 0)
    if settled[0].item() and marker_hit[0].item():
        print(f"[TERMINATION] Lane Departure! LatErr: {lat_err[0].item():.3f}m (Allowed +/- 0.5m)")

    return settled & (chassis_crash | marker_hit)

def height_termination(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    """Terminates if the robot flips or falls."""
    asset = env.scene[asset_cfg.name]
    height = asset.data.root_pos_w[:, 2]
    settled = env.episode_length_buf > 5
    # Adjusted height for 1x car (Min 0.02m to avoid ground clipping, Max 0.5m for airtime)
    return settled & ((height < 0.02) | (height > 0.5))

def stagnation_termination(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    """Terminates if the robot hasn't made forward progress in 100 steps."""
    # Lower settling buffer for light 1x physics
    settled = env.episode_length_buf > 10
    
    # Initialize extras if not present
    if "last_dist" not in env.extras:
        env.extras["last_dist"] = torch.zeros(env.num_envs, device=env.device)
        env.extras["stagnant_steps"] = torch.zeros(env.num_envs, device=env.device)

    current_dist = env.extras.get("distance", torch.zeros(env.num_envs, device=env.device))
    
    # Check progress only if settled
    # (Reset counter if we haven't settled yet to avoid instant reset)
    # Lower progress threshold for 1x scale (0.01m vs 0.1m)
    progress = (current_dist - env.extras["last_dist"]) > 0.01
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
