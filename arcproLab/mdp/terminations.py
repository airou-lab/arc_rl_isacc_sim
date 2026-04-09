# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import torch
from isaaclab.managers import SceneEntityCfg
from isaaclab.envs import ManagerBasedRLEnv

def white_line_contact(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    """
    Terminates if the robot 'contacts' the roadmarks (2D proximity) or physically crashes.
    """
    # 1. Physical Crash (Chassis hitting signs/walls/curbs)
    sensor = env.scene.sensors["contact_forces"]
    
    # Use 5000.0 N for 8x scale to ignore minor physics noise.
    forces = torch.norm(sensor.data.net_forces_w, dim=-1)
    contact_mask = forces > 5000.0
    chassis_crash = torch.any(contact_mask, dim=1)
    
    # 2. Virtual Roadmark Contact (2D Distance)
    # RELAXED FOR CALIBRATION: 2.0m physical distance
    # 2.0m * 0.125 (normalization) = 0.25
    obs = env.observation_manager.compute()["policy"]
    lat_err_normalized = torch.abs(obs[:, 8])
    marker_hit = lat_err_normalized > 0.25

    # Combine with debug logging
    if chassis_crash[0].item():
        print(f"[TERMINATION] Physical Crash! Force: {torch.max(forces[0]):.1f}N")
    if marker_hit[0].item():
        print(f"[TERMINATION] Lane Departure! LatErr Norm: {lat_err_normalized[0]:.4f} (Limit 0.25)")

    return chassis_crash | marker_hit

def height_termination(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    """Terminates if the robot flips or falls (chassis height < 0.1m or > 2.0m for 8x scale)."""
    asset = env.scene[asset_cfg.name]
    height = asset.data.root_pos_w[:, 2]
    
    # Use 5-step settling for height to allow landing, but otherwise strict.
    settled = env.episode_length_buf > 5
    
    return settled & ((height < 0.1) | (height > 2.0))
