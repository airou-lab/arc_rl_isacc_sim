# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import torch
from isaaclab.managers import SceneEntityCfg
from isaaclab.envs import ManagerBasedRLEnv

def white_line_contact(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    """
    Terminates if the robot hits the roadmarks (at 1.5m) or crashes the chassis.
    """
    # 1. Physical Crash (Chassis hitting signs/walls/curbs)
    sensor = env.scene.sensors["contact_forces"]
    contact_mask = torch.norm(sensor.data.net_forces_w, dim=-1) > 1.0
    chassis_crash = torch.any(contact_mask, dim=1)
    
    # 2. Roadmark hit (Exactly 1.5m from centerline)
    # Using normalized obs: 1.5m * 0.125 = 0.1875
    obs = env.observation_manager.compute()["policy"]
    marker_hit = torch.abs(obs[:, 8]) > 0.1875
    
    return chassis_crash | marker_hit

def height_termination(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    """Terminates if the robot flips or falls (chassis height < 0.1m or > 2.0m for 8x scale)."""
    asset = env.scene[asset_cfg.name]
    height = asset.data.root_pos_w[:, 2]
    return (height < 0.1) | (height > 2.0)
