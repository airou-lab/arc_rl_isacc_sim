# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import torch
from isaaclab.managers import SceneEntityCfg
from isaaclab.envs import ManagerBasedRLEnv

def composite_reward(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    """
    Original ARCPro Composite Reward:
    - Speed: speed * 0.3
    - Lateral Error: 1.0 if < 0.5m, else -2.0 * lat_err
    - Collision: -20.0 if termination flag triggered
    """
    asset = env.scene[asset_cfg.name]
    speed = asset.data.root_lin_vel_b[:, 0]
    
    # 1. Speed Reward
    rew_speed = speed * 0.3
    
    # 2. Lateral Error Reward (threshold-based)
    # We pull lat_err from the observation manager (policy group, index 8)
    obs = env.observation_manager.compute()["policy"]
    lat_err = obs[:, 8]
    
    rew_lat = torch.where(
        torch.abs(lat_err) < 0.5,
        torch.ones_like(lat_err),
        -torch.abs(lat_err) * 2.0
    )
    
    # 3. Collision Penalty
    # Triggered by ANY termination term (usually white_line_contact or base_contact)
    terminated = env.termination_manager.terminated
    rew_col = torch.where(terminated, torch.tensor(-20.0, device=env.device), torch.tensor(0.0, device=env.device))
    
    total_reward = rew_speed + rew_lat + rew_col
    
    # Handle NaNs
    nan_mask = torch.isnan(total_reward)
    if nan_mask.any():
        total_reward[nan_mask] = 0.0
        
    return total_reward
