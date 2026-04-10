# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import torch
import gymnasium as gym
from dataclasses import dataclass
from isaaclab.envs import ManagerBasedEnv
from isaaclab.managers.action_manager import ActionTerm, ActionTermCfg
from isaaclab.assets import Articulation
import isaaclab.envs.mdp.actions.actions_cfg as actions_cfg

@dataclass
class GroupedJointActionCfg(actions_cfg.JointActionCfg):
    """Configuration for a grouped joint action term."""
    def __post_init__(self):
        # Set missing class_type for the manager to know which class to instantiate
        if self.class_type is None:
            self.class_type = GroupedJointAction

class GroupedJointAction(ActionTerm):
    """Action term that maps a single scalar action to multiple joints."""
    
    cfg: GroupedJointActionCfg
    _asset: Articulation

    def __init__(self, cfg: GroupedJointActionCfg, env: ManagerBasedEnv) -> None:
        super().__init__(cfg, env)
        # Resolve joints
        self._joint_ids, self._joint_names = self._asset.find_joints(self.cfg.joint_names)
        self._num_joints = len(self._joint_ids)
        
        # Prepare scale and offset tensors (num_envs, num_joints)
        self._scale = torch.tensor(self.cfg.scale, device=self.device).repeat(self.num_envs, self._num_joints)
        self._offset = torch.tensor(self.cfg.offset, device=self.device).repeat(self.num_envs, self._num_joints)
        
        # Action buffers
        self._raw_actions = torch.zeros(self.num_envs, self.action_dim, device=self.device)
        self._processed_actions = torch.zeros(self.num_envs, self._num_joints, device=self.device)

    @property
    def action_dim(self) -> int:
        return 1

    @property
    def raw_actions(self) -> torch.Tensor:
        return self._raw_actions

    @property
    def processed_actions(self) -> torch.Tensor:
        return self._processed_actions

    def process_actions(self, actions: torch.Tensor):
        # actions is (num_envs, 1)
        self._raw_actions[:] = actions
        # Broadcast and apply scale/offset
        self._processed_actions[:] = self._offset + self._scale * actions.repeat(1, self._num_joints)

    def apply_actions(self):
        # To be implemented by child classes
        pass

@dataclass
class GroupedJointPositionActionCfg(GroupedJointActionCfg):
    def __post_init__(self):
        self.class_type = GroupedJointPositionAction

class GroupedJointPositionAction(GroupedJointAction):
    def apply_actions(self):
        self._asset.set_joint_position_target(self._processed_actions, joint_ids=self._joint_ids)

@dataclass
class GroupedJointVelocityActionCfg(GroupedJointActionCfg):
    def __post_init__(self):
        self.class_type = GroupedJointVelocityAction

class GroupedJointVelocityAction(GroupedJointAction):
    def apply_actions(self):
        self._asset.set_joint_velocity_target(self._processed_actions, joint_ids=self._joint_ids)
