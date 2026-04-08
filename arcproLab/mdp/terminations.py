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
    
    # Use 5000.0 N for 8x scale to ignore ground jitter/landing impact.
    forces = torch.norm(sensor.data.net_forces_w, dim=-1)
    contact_mask = forces > 5000.0
    chassis_crash = torch.any(contact_mask, dim=1)
    
    # 2. Virtual Roadmark Contact (2D Distance)
    # Target: 1.2m physical distance (Standard lane half-width for 8x robot center)
    # 1.2m * 0.125 (normalization) = 0.15
    obs = env.observation_manager.compute()["policy"]
    lat_err_normalized = torch.abs(obs[:, 8])
    marker_hit = lat_err_normalized > 0.15
    
    # Settling Time: Ignore ALL terminations for the first 100 steps of an episode
    # This allows the 8x scale asset to land and stabilize its physics/observations.
    settled = env.episode_length_buf > 100
    
    # Combine with debug logging
    if settled.any():
        if chassis_crash.any():
            print(f"[TERMINATION] Physical Crash! Force: {torch.max(forces):.1f}N")
        if marker_hit.any():
            print(f"[TERMINATION] Lane Departure! LatErr Norm: {lat_err_normalized[0]:.3f} (Limit 0.15)")

    # Only terminate if we are 'settled'
    return settled & (chassis_crash | marker_hit)

def height_termination(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    """Terminates if the robot flips or falls (chassis height < 0.1m or > 2.0m for 8x scale)."""
    asset = env.scene[asset_cfg.name]
    height = asset.data.root_pos_w[:, 2]
    return (height < 0.1) | (height > 2.0)
