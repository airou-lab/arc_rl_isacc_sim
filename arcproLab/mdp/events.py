# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import torch
from isaaclab.managers import SceneEntityCfg
from isaaclab.envs import ManagerBasedRLEnv

def reset_robot_to_lane(env: ManagerBasedRLEnv, env_ids: torch.Tensor, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")):
    """
    Event to reset the robot to the nearest lane center.
    """
    asset = env.scene[asset_cfg.name]
    
    # Aligned spawn: Nearest WP [-1.36, -0.33, -1.606]
    # Set Z to 0.1 and shifted right (X decreased to -1.56)
    pos = torch.tensor([[-1.56, -0.33, 0.1]], device=env.device).repeat(len(env_ids), 1)
    # Yaw -1.606 rad to quaternion (w, x, y, z)
    quat = torch.tensor([[0.696, 0.0, 0.0, -0.718]], device=env.device).repeat(len(env_ids), 1)
    
    asset.write_root_pose_to_sim(torch.cat([pos, quat], dim=-1), env_ids=env_ids)
    asset.write_root_velocity_to_sim(torch.zeros((len(env_ids), 6), device=env.device), env_ids=env_ids)
