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
class ThrottleBrakeVelocityActionCfg(GroupedJointActionCfg):
    """Throttle + brake combined into a single velocity-actuating term.

    Consumes a 2-D action [throttle, brake], both clipped to [0, 1], and
    produces one wheel-velocity target per joint. Brake attenuates throttle
    multiplicatively, matching the policy's isaac_direct_env semantics:

        wheel_vel_target = scale * throttle * (1 - brake)

    The outer ActionCfg still exposes a 3-channel [steer, throttle, brake]
    contract to the policy: this term contributes the throttle (idx 1) and
    brake (idx 2) slots; the brake actuator is implemented as a derate on
    the throttle command, not as an independent decelerating torque.
    """
    def __post_init__(self):
        # Set unconditionally (matches the sibling cfg pattern in this module).
        # The `if is None` guard would skip when class_type holds IsaacLab's
        # MISSING sentinel rather than None, leaving validate() to fail.
        self.class_type = ThrottleBrakeVelocityAction


class ThrottleBrakeVelocityAction(GroupedJointAction):
    """2-D throttle+brake → wheel-velocity, broadcast across drive joints."""

    cfg: ThrottleBrakeVelocityActionCfg

    @property
    def action_dim(self) -> int:
        return 2  # [throttle, brake]

    @property
    def action_space(self) -> gym.Space:
        return gym.spaces.Box(low=0.0, high=1.0, shape=(self.action_dim,))

    def process_actions(self, actions: torch.Tensor):
        # actions: (num_envs, 2) = [throttle, brake]
        self._raw_actions[:] = actions
        throttle = torch.clamp(actions[:, 0:1], 0.0, 1.0)
        brake    = torch.clamp(actions[:, 1:2], 0.0, 1.0)
        # T3.3 go_signal action gate. When the GoSignalManager places this
        # env in STOP (bar detected within stop_distance_threshold),
        # env.extras["go_signal"] is 0 and we force throttle to 0 here at
        # the env boundary regardless of what the network commanded. This
        # is a hard safety gate, not reward shaping. The policy still sees
        # go_signal in slot 1 of its telemetry obs so it can learn to issue
        # throttle=0 itself, but the gate guarantees the behavior even
        # mid-learning.
        try:
            go_signal = self._env.extras.get("go_signal", None)
            if go_signal is not None:
                throttle = throttle * go_signal.unsqueeze(-1).to(throttle.dtype)
        except Exception:
            pass
        drive    = throttle * (1.0 - brake)        # (num_envs, 1)
        self._processed_actions[:] = self._offset + self._scale * drive.repeat(1, self._num_joints)

    def apply_actions(self):
        self._asset.set_joint_velocity_target(self._processed_actions, joint_ids=self._joint_ids)
