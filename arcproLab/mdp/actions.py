from dataclasses import dataclass, field

import gymnasium as gym
import torch

try:
    import isaaclab.envs.mdp.actions.actions_cfg as actions_cfg
    from isaaclab.assets import Articulation
    from isaaclab.envs import ManagerBasedEnv
    from isaaclab.managers.action_manager import ActionTerm, ActionTermCfg
    _JOINT_ACTION_CFG_BASE = actions_cfg.JointActionCfg
except ImportError:
    Articulation = object  # type: ignore[misc,assignment]
    ManagerBasedEnv = object  # type: ignore[misc,assignment]

    @dataclass
    class ActionTermCfg:  # type: ignore[no-redef]
        class_type: type | None = None
        asset_name: str = "robot"

    @dataclass
    class _JointActionCfgStub(ActionTermCfg):
        joint_names: list[str] = field(default_factory=list)
        scale: float = 1.0
        offset: float = 0.0
        clip: dict | tuple | None = None

    _JOINT_ACTION_CFG_BASE = _JointActionCfgStub

    class ActionTerm:  # type: ignore[no-redef]
        def __init__(self, cfg, env):
            self.cfg = cfg
            self._env = env
            self.device = getattr(env, "device", "cpu")
            self.num_envs = getattr(env, "num_envs", 1)
            self._asset = env.scene[cfg.asset_name] if hasattr(env, "scene") else None


def get_normalized_actions(env) -> torch.Tensor:
    """Return normalized (num_envs, 2) [steer, drive] actions strictly in [-1.0, 1.0].

    Prefers clamped term-level normalized actions (`a_steering`, `b_drive`), and un-offsets /
    un-scales any raw target values before clamping to [-1.0, 1.0].
    """
    num_envs = getattr(env, "num_envs", 1)
    device = getattr(env, "device", "cpu")
    out = torch.zeros((num_envs, 2), device=device, dtype=torch.float32)

    am = getattr(env, "action_manager", None)
    if am is None:
        return out

    # 1. Try reading from explicit steering and drive terms first
    try:
        if hasattr(am, "get_term"):
            steer_term = am.get_term("a_steering")
            drive_term = am.get_term("b_drive")
            if steer_term is not None and hasattr(steer_term, "normalized_actions"):
                out[:, 0] = steer_term.normalized_actions.view(-1)
            elif steer_term is not None and hasattr(steer_term, "raw_actions"):
                out[:, 0] = torch.clamp(
                    steer_term.raw_actions.view(-1), min=-1.0, max=1.0
                )
            if drive_term is not None and hasattr(drive_term, "normalized_actions"):
                out[:, 1] = drive_term.normalized_actions.view(-1)
            elif drive_term is not None and hasattr(drive_term, "raw_actions"):
                out[:, 1] = torch.clamp(
                    drive_term.raw_actions.view(-1), min=-1.0, max=1.0
                )
            return torch.clamp(out, min=-1.0, max=1.0)
    except Exception:
        pass

    # 2. Fallback to action_manager.action buffer
    try:
        raw = getattr(am, "action", None)
        if raw is not None and raw.shape[1] >= 2:
            steer_col = raw[:, 0]
            drive_col = raw[:, 1]
            # If drive_col looks like unnormalized wheel velocity targets (e.g., [0, 40] rad/s
            # from scale=20.0, offset=20.0), un-offset and normalize back to [-1.0, 1.0].
            if (drive_col.abs() > 1.5).any():
                drive_col = (drive_col - 20.0) / 20.0
            out[:, 0] = torch.clamp(steer_col, min=-1.0, max=1.0)
            out[:, 1] = torch.clamp(drive_col, min=-1.0, max=1.0)
    except Exception:
        pass

    return out


@dataclass
class GroupedJointActionCfg(_JOINT_ACTION_CFG_BASE):
    """Configuration for a grouped joint action term."""

    invert_right_joints: bool = False

    def __post_init__(self):
        if self.class_type is None:
            self.class_type = GroupedJointAction


class GroupedJointAction(ActionTerm):
    """Action term that maps a single scalar action to multiple joints."""

    cfg: GroupedJointActionCfg
    _asset: Articulation

    def __init__(self, cfg: GroupedJointActionCfg, env: ManagerBasedEnv) -> None:
        super().__init__(cfg, env)
        self._joint_ids, self._joint_names = self._asset.find_joints(
            self.cfg.joint_names
        )
        self._num_joints = len(self._joint_ids)

        self._scale = torch.tensor(
            self.cfg.scale, device=self.device, dtype=torch.float32
        ).repeat(self.num_envs, self._num_joints)
        self._offset = torch.tensor(
            self.cfg.offset, device=self.device, dtype=torch.float32
        ).repeat(self.num_envs, self._num_joints)

        self._raw_actions = torch.zeros(
            self.num_envs, self.action_dim, device=self.device, dtype=torch.float32
        )
        self._processed_actions = torch.zeros(
            self.num_envs, self._num_joints, device=self.device, dtype=torch.float32
        )

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
        low, high = -1.0, 1.0
        if hasattr(self.cfg, "clip") and self.cfg.clip is not None:
            if isinstance(self.cfg.clip, dict):
                for val in self.cfg.clip.values():
                    low, high = val[0], val[1]
            elif isinstance(self.cfg.clip, tuple):
                low, high = self.cfg.clip[0], self.cfg.clip[1]

        return gym.spaces.Box(low=low, high=high, shape=(self.action_dim,))

    @property
    def raw_actions(self) -> torch.Tensor:
        return self._raw_actions

    @property
    def normalized_actions(self) -> torch.Tensor:
        return self._raw_actions

    @property
    def processed_actions(self) -> torch.Tensor:
        return self._processed_actions

    def unscale_processed_actions(self, processed: torch.Tensor) -> torch.Tensor:
        """Recover normalized [-1.0, 1.0] command from scaled/offset joint targets."""
        safe_scale = torch.where(
            torch.abs(self._scale) > 1e-6,
            self._scale,
            torch.ones_like(self._scale),
        )
        unscaled = (processed - self._offset) / safe_scale
        return torch.clamp(unscaled.mean(dim=-1, keepdim=True), min=-1.0, max=1.0)

    def process_actions(self, actions: torch.Tensor):
        if hasattr(self.cfg, "clip") and self.cfg.clip is not None:
            if isinstance(self.cfg.clip, dict):
                for val in self.cfg.clip.values():
                    actions = torch.clamp(actions, val[0], val[1])
            elif isinstance(self.cfg.clip, tuple):
                actions = torch.clamp(actions, self.cfg.clip[0], self.cfg.clip[1])

        # Clamp normalized actions strictly to [-1.0, 1.0] before storing and scaling
        actions = torch.clamp(actions, min=-1.0, max=1.0)
        self._raw_actions[:] = actions

        self._processed_actions[:] = self._offset + self._scale * actions.repeat(
            1, self._num_joints
        )

        if (
            getattr(self.cfg, "invert_right_joints", False)
            and len(self._right_wheel_indices) > 0
        ):
            self._processed_actions[:, self._right_wheel_indices] *= -1.0

    def apply_actions(self):
        pass


@dataclass
class AckermannSteeringActionCfg(ActionTermCfg):
    """Configuration for Ackermann steering action term."""

    asset_name: str = "robot"
    joint_names: list[str] = field(default_factory=list)
    wheelbase: float = 0.33
    track_width: float = 0.28
    max_steering_angle: float = 0.68
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
        self._joint_ids, self._joint_names = self._asset.find_joints(
            self.cfg.joint_names
        )
        if len(self._joint_ids) != 2:
            raise ValueError(
                f"AckermannSteeringAction requires exactly 2 joints, found {len(self._joint_ids)}: {self._joint_names}"
            )

        self._left_idx = self._joint_names.index(self.cfg.joint_names[0])
        self._right_idx = self._joint_names.index(self.cfg.joint_names[1])

        self.wheelbase = cfg.wheelbase
        self.half_track = cfg.track_width / 2.0

        self._raw_actions = torch.zeros(
            self.num_envs, 1, device=self.device, dtype=torch.float32
        )
        self._steering_angle = torch.zeros(
            self.num_envs, device=self.device, dtype=torch.float32
        )
        self._processed_actions = torch.zeros(
            self.num_envs, 2, device=self.device, dtype=torch.float32
        )

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
    def normalized_actions(self) -> torch.Tensor:
        return self._raw_actions

    @property
    def steering_angle(self) -> torch.Tensor:
        return self._steering_angle

    @property
    def processed_actions(self) -> torch.Tensor:
        return self._processed_actions

    def process_actions(self, actions: torch.Tensor):
        clamped_actions = torch.clamp(actions, min=-1.0, max=1.0)
        self._raw_actions[:] = clamped_actions

        max_steer = float(self.cfg.max_steering_angle)
        steering_angle = torch.clamp(
            clamped_actions.squeeze(-1) * max_steer + float(self.cfg.offset),
            min=-max_steer,
            max=max_steer,
        )
        self._steering_angle[:] = steering_angle

        is_straight = torch.abs(steering_angle) < 1e-4

        turn_radius = self.wheelbase / torch.tan(torch.abs(steering_angle) + 1e-6)
        inner_angle = torch.atan(self.wheelbase / (turn_radius - self.half_track))
        outer_angle = torch.atan(self.wheelbase / (turn_radius + self.half_track))

        # Exact Left/Right odd mirror symmetry:
        # steering_angle > 0 (Left turn): Left is Inner (+inner), Right is Outer (+outer)
        # steering_angle < 0 (Right turn): Left is Outer (-outer), Right is Inner (-inner)
        left_angle = torch.where(steering_angle > 0, inner_angle, -outer_angle)
        right_angle = torch.where(steering_angle > 0, outer_angle, -inner_angle)

        left_angle = torch.where(is_straight, torch.zeros_like(left_angle), left_angle)
        right_angle = torch.where(
            is_straight, torch.zeros_like(right_angle), right_angle
        )

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

        # Retrieve clamped, offset-aware steering angle from steering action term
        steering_angle = None
        try:
            if hasattr(self._env, "action_manager"):
                steer_term = self._env.action_manager.get_term(
                    self.cfg.steering_term_name
                )
                if (
                    hasattr(steer_term, "steering_angle")
                    and steer_term.steering_angle is not None
                ):
                    steering_angle = steer_term.steering_angle
                elif (
                    hasattr(steer_term, "raw_actions")
                    and steer_term.raw_actions is not None
                ):
                    max_steer = float(
                        getattr(steer_term.cfg, "max_steering_angle", 0.68)
                    )
                    offset = float(getattr(steer_term.cfg, "offset", 0.0))
                    clamped_steer = torch.clamp(
                        steer_term.raw_actions.squeeze(-1), min=-1.0, max=1.0
                    )
                    steering_angle = torch.clamp(
                        clamped_steer * max_steer + offset,
                        min=-max_steer,
                        max=max_steer,
                    )
        except Exception:
            pass

        if steering_angle is None:
            try:
                if (
                    hasattr(self._env, "action_manager")
                    and self._env.action_manager.action is not None
                ):
                    clamped_steer = torch.clamp(
                        self._env.action_manager.action[:, 0], min=-1.0, max=1.0
                    )
                    steering_angle = clamped_steer * 0.68
            except Exception:
                pass

        if steering_angle is not None:
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
        return 2

    @property
    def action_space(self) -> gym.Space:
        return gym.spaces.Box(low=0.0, high=1.0, shape=(self.action_dim,))

    def process_actions(self, actions: torch.Tensor):
        actions = torch.clamp(actions, 0.0, 1.0)
        self._raw_actions[:] = actions
        throttle = torch.clamp(actions[:, 0:1], 0.0, 1.0)
        brake = torch.clamp(actions[:, 1:2], 0.0, 1.0)

        drive = throttle * (1.0 - brake)

        self._processed_actions[:] = self._offset + self._scale * drive.repeat(
            1, self._num_joints
        )

    def apply_actions(self):
        self._asset.set_joint_velocity_target(
            self._processed_actions, joint_ids=self._joint_ids
        )


import sys as _sys  # noqa: E402

_sys.modules.setdefault("mdp.actions", _sys.modules[__name__])
_sys.modules.setdefault("arcproLab.mdp.actions", _sys.modules[__name__])

