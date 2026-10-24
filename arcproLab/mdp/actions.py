from dataclasses import dataclass, field

import gymnasium as gym
import isaaclab.envs.mdp.actions.actions_cfg as actions_cfg
import torch
from isaaclab.assets import Articulation
from isaaclab.envs import ManagerBasedEnv
from isaaclab.managers.action_manager import ActionTerm, ActionTermCfg


@dataclass
class GroupedJointActionCfg(actions_cfg.JointActionCfg):
    """Configuration for a grouped joint action term."""

    invert_right_joints: bool = False

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
        self._joint_ids, self._joint_names = self._asset.find_joints(
            self.cfg.joint_names
        )
        self._num_joints = len(self._joint_ids)

        # Prepare scale and offset tensors (num_envs, num_joints)
        self._scale = torch.tensor(self.cfg.scale, device=self.device).repeat(
            self.num_envs, self._num_joints
        )
        self._offset = torch.tensor(self.cfg.offset, device=self.device).repeat(
            self.num_envs, self._num_joints
        )

        # Action buffers
        self._raw_actions = torch.zeros(
            self.num_envs, self.action_dim, device=self.device
        )
        self._processed_actions = torch.zeros(
            self.num_envs, self._num_joints, device=self.device
        )

        # Pre-compute right wheel indices for mirroring
        self._right_wheel_indices = [
            i
            for i, name in enumerate(self._joint_names)
            if name.endswith("_R") or name.endswith("_RR") or name.endswith("_FR")
        ]

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
        # FORCE clamp raw actions to [-1.0, 1.0] to protect physics
        actions = torch.clamp(actions, min=-1.0, max=1.0)

        # Broadcast and apply scale/offset
        self._processed_actions[:] = self._offset + self._scale * actions.repeat(
            1, self._num_joints
        )

        # Invert right-side joints if they are physically mirrored in the USD
        if (
            getattr(self.cfg, "invert_right_joints", False)
            and len(self._right_wheel_indices) > 0
        ):
            self._processed_actions[:, self._right_wheel_indices] *= -1.0

    def apply_actions(self):
        # To be implemented by child classes
        pass


@dataclass
class AckermannSteeringActionCfg(ActionTermCfg):
    """Configuration for Ackermann steering action term."""

    asset_name: str = "robot"
    joint_names: list[str] = field(default_factory=list)
    wheelbase: float = 0.33
    track_width: float = 0.28
    max_steering_angle: float = (
        0.68  # 38.96 deg (enables R_min = 0.408m for Corner 1 R=0.464m)
    )
    scale: float = 1.0
    offset: float = 0.0
    invert_right_joint: bool = False

    def __post_init__(self):
        self.class_type = AckermannSteeringAction


class AckermannSteeringAction(ActionTerm):
    """
    Action term that maps a single steering command to two joints using Ackermann geometry.
    Expected joint_names: [Left_Steer, Right_Steer]
    """

    cfg: AckermannSteeringActionCfg
    _asset: Articulation

    def __init__(self, cfg: AckermannSteeringActionCfg, env: ManagerBasedEnv) -> None:
        super().__init__(cfg, env)
        # Resolve joints
        self._joint_ids, self._joint_names = self._asset.find_joints(
            self.cfg.joint_names
        )
        if len(self._joint_ids) != 2:
            raise ValueError(
                f"AckermannSteeringAction requires exactly 2 joints, found {len(self._joint_ids)}: {self._joint_names}"
            )

        # Explicitly map which index corresponds to which requested joint to prevent sorting bugs
        self._left_idx = self._joint_names.index(self.cfg.joint_names[0])
        self._right_idx = self._joint_names.index(self.cfg.joint_names[1])

        # Constants
        self.wheelbase = cfg.wheelbase
        self.half_track = cfg.track_width / 2.0

        # Action buffers
        self._raw_actions = torch.zeros(self.num_envs, 1, device=self.device)
        self._processed_actions = torch.zeros(self.num_envs, 2, device=self.device)

    @property
    def action_dim(self) -> int:
        return 1

    @property
    def action_space(self) -> gym.Space:
        return gym.spaces.Box(low=-1.0, high=1.0, shape=(self.action_dim,))

    @property
    def raw_actions(self) -> torch.Tensor:
        return self._raw_actions

    @property
    def processed_actions(self) -> torch.Tensor:
        return self._processed_actions

    def process_actions(self, actions: torch.Tensor):
        # actions is (num_envs, 1) in range [-1, 1]
        self._raw_actions[:] = actions

        # Clamp raw network actions to [-1.0, 1.0] in case the policy outputs out-of-bound values
        clamped_actions = torch.clamp(actions.squeeze(1), min=-1.0, max=1.0)

        # Convert normalized action to target steering angle (rad) and apply offset
        steering_angle = clamped_actions * self.cfg.max_steering_angle + self.cfg.offset

        # Handle Straight case (to avoid division by zero)
        is_straight = torch.abs(steering_angle) < 1e-4

        # Ackermann Logic (Vectorized)
        # turn_radius = wheelbase / tan(abs(steering_angle))
        turn_radius = self.wheelbase / torch.tan(torch.abs(steering_angle) + 1e-6)

        # inner_angle = atan(wheelbase / (turn_radius - half_track))
        # outer_angle = atan(wheelbase / (turn_radius + half_track))
        inner_angle = torch.atan(self.wheelbase / (turn_radius - self.half_track))
        outer_angle = torch.atan(self.wheelbase / (turn_radius + self.half_track))

        # Map to Left/Right joints
        # If steering_angle > 0 (Turning LEFT): Left is Inner, Right is Outer
        # If steering_angle < 0 (Turning RIGHT): Left is Outer, Right is Inner
        left_angle = torch.where(steering_angle > 0, inner_angle, -outer_angle)
        right_angle = torch.where(steering_angle > 0, outer_angle, -inner_angle)

        # Apply Straight override
        left_angle = torch.where(is_straight, torch.zeros_like(left_angle), left_angle)
        right_angle = torch.where(
            is_straight, torch.zeros_like(right_angle), right_angle
        )

        # Store processed actions mapped explicitly to their actual indices
        self._processed_actions[:, self._left_idx] = left_angle
        self._processed_actions[:, self._right_idx] = right_angle

        if self.cfg.invert_right_joint:
            self._processed_actions[:, self._right_idx] = -self._processed_actions[
                :, self._right_idx
            ]

    def apply_actions(self):
        self._asset.set_joint_position_target(
            self._processed_actions, joint_ids=self._joint_ids
        )


@dataclass
class GroupedJointVelocityActionCfg(GroupedJointActionCfg):
    def __post_init__(self):
        self.class_type = GroupedJointVelocityAction


class GroupedJointVelocityAction(GroupedJointAction):
    def process_actions(self, actions: torch.Tensor):
        super().process_actions(actions)

    def apply_actions(self):
        self._asset.set_joint_velocity_target(
            self._processed_actions, joint_ids=self._joint_ids
        )


@dataclass
class AckermannDifferentialVelocityActionCfg(GroupedJointActionCfg):
    """Configuration for Ackermann Electronic Differential drive action term."""

    wheelbase: float = 0.33
    track_width: float = 0.28
    steering_term_name: str = "a_steering"

    def __post_init__(self):
        self.class_type = AckermannDifferentialVelocityAction


class AckermannDifferentialVelocityAction(GroupedJointAction):
    """
    Action term that applies an electronic differential to drive joints based on steering angle.
    Distributes drive velocity to left and right wheels to eliminate the parasitic locked-spool
    counter-yaw moment during cornering:
        v_outer = v * (1 + W * tan(delta) / (2 * L))
        v_inner = v * (1 - W * tan(delta) / (2 * L))
    Maintains constant average forward speed while matching the exact kinematic arc speeds.
    """

    cfg: AckermannDifferentialVelocityActionCfg

    def __init__(
        self, cfg: AckermannDifferentialVelocityActionCfg, env: ManagerBasedEnv
    ) -> None:
        super().__init__(cfg, env)
        self.wheelbase = cfg.wheelbase
        self.half_track = cfg.track_width / 2.0

        # Classify which joints are on the left side vs right side
        # Joint names typically: Joint_Drive_FL, Joint_Drive_FR, Joint_Drive_RL, Joint_Drive_RR
        self._left_joint_indices = []
        self._right_joint_indices = []
        for i, name in enumerate(self._joint_names):
            if (
                name.endswith("_L")
                or name.endswith("L")
                or "_L_" in name
                or "Left" in name
            ):
                self._left_joint_indices.append(i)
            elif (
                name.endswith("_R")
                or name.endswith("R")
                or "_R_" in name
                or "Right" in name
            ):
                self._right_joint_indices.append(i)
            else:
                self._left_joint_indices.append(i)

    def process_actions(self, actions: torch.Tensor):
        super().process_actions(actions)

        # Retrieve active steering angle from steering action term or raw action buffer
        steering_angle = None
        try:
            if hasattr(self._env, "action_manager"):
                steer_term = self._env.action_manager.get_term(
                    self.cfg.steering_term_name
                )
                if (
                    hasattr(steer_term, "raw_actions")
                    and steer_term.raw_actions is not None
                ):
                    max_steer = getattr(steer_term.cfg, "max_steering_angle", 0.68)
                    steering_angle = steer_term.raw_actions.squeeze(-1) * max_steer
        except Exception:
            pass

        if steering_angle is None:
            try:
                if (
                    hasattr(self._env, "action_manager")
                    and self._env.action_manager.action is not None
                ):
                    steering_angle = self._env.action_manager.action[:, 0] * 0.68
            except Exception:
                pass

        if steering_angle is not None:
            # Kinematic speed ratio: diff_factor = (W * tan(delta)) / (2 * L)
            # delta > 0: Left turn -> Left wheels inner (slower), Right wheels outer (faster)
            # delta < 0: Right turn -> Left wheels outer (faster), Right wheels inner (slower)
            tan_delta = torch.tan(steering_angle)
            diff_factor = (self.half_track * tan_delta) / self.wheelbase
            diff_factor = torch.clamp(diff_factor, min=-0.60, max=0.60).unsqueeze(1)

            left_mult = 1.0 - diff_factor
            right_mult = 1.0 + diff_factor

            if len(self._left_joint_indices) > 0 and len(self._right_joint_indices) > 0:
                self._processed_actions[:, self._left_joint_indices] *= left_mult
                self._processed_actions[:, self._right_joint_indices] *= right_mult

    def apply_actions(self):
        self._asset.set_joint_velocity_target(
            self._processed_actions, joint_ids=self._joint_ids
        )


@dataclass
class CombinedDriveActionCfg(GroupedJointActionCfg):
    """Configuration for a combined drive action term (Throttle + Brake)."""

    def __post_init__(self):
        self.class_type = CombinedDriveAction


class CombinedDriveAction(GroupedJointAction):
    """
    Action term that fuses two scalar actions (Throttle, Brake) into a single
    velocity command for multiple joints.

    Formula: velocity = scale * throttle * (1.0 - brake) + offset
    """

    @property
    def action_dim(self) -> int:
        return 2  # [Throttle, Brake]

    @property
    def action_space(self) -> gym.Space:
        return gym.spaces.Box(low=0.0, high=1.0, shape=(self.action_dim,))

    def process_actions(self, actions: torch.Tensor):
        # actions is (num_envs, 2) -> [throttle, brake]
        # Explicitly clamp to [0, 1] to respect the action space
        actions = torch.clamp(actions, 0.0, 1.0)
        self._raw_actions[:] = actions
        throttle = torch.clamp(actions[:, 0:1], 0.0, 1.0)
        brake = torch.clamp(actions[:, 1:2], 0.0, 1.0)

        drive = throttle * (1.0 - brake)

        # Broadcast and apply scale/offset to all drive joints
        self._processed_actions[:] = self._offset + self._scale * drive.repeat(
            1, self._num_joints
        )

    def apply_actions(self):
        self._asset.set_joint_velocity_target(
            self._processed_actions, joint_ids=self._joint_ids
        )
