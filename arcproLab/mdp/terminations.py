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
    
    # CALIBRATION (Step 0 Center = 0.0m)
    # If robot is 3.6m wide, it has only ~0.5m of wiggle room in a 4.5m lane.
    # Reset if it drifts more than 0.6m from its starting lane center.
    drift = torch.abs(lat_err)
    marker_hit = drift > 0.6

    # Apply 20-step settling buffer for the physics to 'link' (8x scale is heavy)
    settled = env.episode_length_buf > 20

    # Debug logging (env 0)
    if settled[0].item() and marker_hit[0].item():
        print(f"[TERMINATION] Lane Departure! Drift: {drift[0].item():.3f}m (Limit 0.6m) | Raw LatErr: {lat_err[0].item():.3f}m")

    return settled & (chassis_crash | marker_hit)

def height_termination(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    """Terminates if the robot flips or falls."""
    asset = env.scene[asset_cfg.name]
    height = asset.data.root_pos_w[:, 2]
    settled = env.episode_length_buf > 5
    return settled & ((height < 0.1) | (height > 2.0))
