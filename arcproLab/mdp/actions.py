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
    def action_space(self) -> gym.Space:
        # Define bounds based on clip if available
        low, high = -1.0, 1.0
        if hasattr(self.cfg, "clip") and self.cfg.clip is not None:
            if isinstance(self.cfg.clip, dict):
                # For grouped actions, we usually just have one term
                for val in self.cfg.clip.values():
                    low, high = val[0], val[1]
            elif isinstance(self.cfg.clip, tuple):
                low, high = self.cfg.clip[0], self.cfg.clip[1]
        
        return gym.spaces.Box(low=low, high=high, shape=(self.action_dim,))

    @property
    def raw_actions(self) -> torch.Tensor:
        return self._raw_actions

    @property
    def processed_actions(self) -> torch.Tensor:
        return self._processed_actions

    def process_actions(self, actions: torch.Tensor):
        # actions is (num_envs, 1)
        self._raw_actions[:] = actions
        
        # Apply clipping if specified in config
        # Note: self.cfg.clip can be a dict or None
        if hasattr(self.cfg, "clip") and self.cfg.clip is not None:
            if isinstance(self.cfg.clip, dict):
                # For grouped actions, we usually just have one term, but we check all
                for val in self.cfg.clip.values():
                    actions = torch.clamp(actions, val[0], val[1])
            elif isinstance(self.cfg.clip, tuple):
                actions = torch.clamp(actions, self.cfg.clip[0], self.cfg.clip[1])

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


@dataclass
class NoOpBrakeActionCfg(ActionTermCfg):
    """Configuration for a no-op brake action term.

    Declares a width-1 action channel that is clamped to [0, 1] and discarded.
    Lets the sim expose the policy repo's 3-channel action contract
    (steer, throttle, brake) before the actual brake actuator is wired
    (OM 2-B deferred to V2). asset_name is required by the action manager's
    asset resolution but the term touches no joints.
    """
    def __post_init__(self):
        if self.class_type is None:
            self.class_type = NoOpBrakeAction


class NoOpBrakeAction(ActionTerm):
    """Absorbs a single brake channel and discards it.

    The brake channel is accepted into the action vector so policies trained
    against a 3-D action space don't shape-mismatch the sim, but no joint
    command is emitted.
    """
    cfg: NoOpBrakeActionCfg

    def __init__(self, cfg: NoOpBrakeActionCfg, env: ManagerBasedEnv) -> None:
        super().__init__(cfg, env)
        self._raw_actions = torch.zeros(self.num_envs, 1, device=self.device)
        self._processed_actions = torch.zeros(self.num_envs, 1, device=self.device)

    @property
    def action_dim(self) -> int:
        return 1

    @property
    def action_space(self) -> gym.Space:
        return gym.spaces.Box(low=0.0, high=1.0, shape=(1,))

    @property
    def raw_actions(self) -> torch.Tensor:
        return self._raw_actions

    @property
    def processed_actions(self) -> torch.Tensor:
        return self._processed_actions

    def process_actions(self, actions: torch.Tensor):
        self._raw_actions[:] = actions
        self._processed_actions[:] = torch.clamp(actions, 0.0, 1.0)

    def apply_actions(self):
        pass
