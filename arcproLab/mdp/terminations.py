# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import torch
from isaaclab.managers import SceneEntityCfg
from isaaclab.envs import ManagerBasedRLEnv

def white_line_contact(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    """Terminates if any part of the robot contacts the white lines (track boundaries)."""
    # contact_forces manager has filtered prim paths for the track
    # net_contact_forces is (num_envs, num_bodies, 3)
    # We check if the magnitude of the contact force is > 0
    contact_forces = env.scene.sensors["contact_forces"].data.net_forces_w
    return torch.any(torch.norm(contact_forces, dim=-1) > 1.0, dim=1)

def height_termination(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    """Terminates if the robot flips or falls (chassis height < 0.02m or > 0.3m)."""
    asset = env.scene[asset_cfg.name]
    height = asset.data.root_pos_w[:, 2]
    return (height < 0.02) | (height > 0.3)
