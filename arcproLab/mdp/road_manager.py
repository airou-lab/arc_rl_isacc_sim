# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import os
import torch

try:
    from mdp.track_topology import (
        in_any_corner,
        in_bend_turn,
        in_c1_turn,
        in_c2_turn,
        in_c4_turn,
        in_multi_branch_intersection,
    )
except ImportError:
    from arcproLab.mdp.track_topology import (
        in_any_corner,
        in_bend_turn,
        in_c1_turn,
        in_c2_turn,
        in_c4_turn,
        in_multi_branch_intersection,
    )


class RoadManager:
    """
    Vectorized Road Manager for Multi-Agent & Multi-Branch Intersection Navigation.
    Replaces the singleton RoadGraph to support independent per-environment agent state
    and command-conditioned turn tokens (`-1.0 = LEFT`, `0.0 = STRAIGHT`, `+1.0 = RIGHT`).

    State Shape: (num_envs, num_agents)
    """

    CRUISING = 0
    DECIDING = 1
    COMMITTED = 2

    def __init__(self, num_envs: int, num_agents: int, device: str = "cuda:0"):
        self.device = device
        self.num_envs = num_envs
        self.num_agents = num_agents

        # Turn Tokens: -1.0 = LEFT, 0.0 = STRAIGHT, +1.0 = RIGHT
        self.turn_tokens = torch.zeros((num_envs, num_agents), device=self.device)
        self.go_signals = torch.ones((num_envs, num_agents), device=self.device)

        # Multi-branch & turn-token randomization state (Gate 3)
        self.randomize_turn_tokens: bool = os.environ.get(
            "ARCPRO_RANDOMIZE_TURN_TOKENS", "0"
        ).strip() in ("1", "true", "True")
        self.multi_branch_enabled: bool = self.randomize_turn_tokens or (
            os.environ.get("ARCPRO_MULTI_BRANCH", "0").strip() in ("1", "true", "True")
        )
        self.allowed_tokens = torch.tensor(
            [-1.0, 0.0, 1.0], device=self.device, dtype=torch.float32
        )
        self.mission_tokens = torch.full(
            (num_envs, num_agents), -1.0, device=self.device, dtype=torch.float32
        )
        self.active_branch_ids = torch.zeros(
            num_envs, device=self.device, dtype=torch.long
        )
        self.fsm_state = torch.zeros(
            num_envs, device=self.device, dtype=torch.long
        )
        self.exit_correct = torch.zeros(
            num_envs, device=self.device, dtype=torch.float32
        )
        self.just_exited = torch.zeros(
            num_envs, device=self.device, dtype=torch.bool
        )
        self._in_inter_prev = torch.zeros(
            num_envs, device=self.device, dtype=torch.bool
        )
        self._branch_latched = torch.zeros(
            num_envs, device=self.device, dtype=torch.bool
        )

        # Initialize missions
        self.randomize_missions(
            torch.ones(num_envs, dtype=torch.bool, device=self.device)
        )

        # Gate discovery
        self.gates_pos = None
        self.gates_intent = None
        self.initialized = False

    def _initialize_gates(self):
        if "omni.usd" not in _sys.modules:
            self.initialized = True
            return
        try:
            import omni.usd
            from pxr import Usd, UsdGeom

            stage = omni.usd.get_context().get_stage()
            if not stage:
                self.initialized = True
                return
        except ImportError:
            self.initialized = True
            return

        gate_positions = []
        gate_intents = []

        for prim in stage.Traverse():
            if "laneGate" in prim.GetName():
                xform = UsdGeom.Xformable(prim)
                world_transform = xform.ComputeLocalToWorldTransform(
                    Usd.TimeCode.Default()
                )
                pos = world_transform.ExtractTranslation()

                intent = 0.0
                attr = prim.GetAttribute("SignalTurnRelation")
                if attr.IsValid():
                    val = attr.Get()
                    if isinstance(val, str):
                        if val == "Left":
                            intent = -1.0
                        elif val == "Right":
                            intent = 1.0
                        else:
                            intent = 0.0
                    else:
                        intent = float(val)

                gate_positions.append([pos[0], pos[1]])
                gate_intents.append(intent)

        if gate_positions:
            self.gates_pos = torch.tensor(
                gate_positions, device=self.device, dtype=torch.float32
            )
            self.gates_intent = torch.tensor(
                gate_intents, device=self.device, dtype=torch.float32
            )
            print(f"[RoadManager] Discovered {len(gate_positions)} navigation gates.")

        self.initialized = True

    @staticmethod
    def _tokens_to_branch_ids(tokens_1d: torch.Tensor) -> torch.Tensor:
        """Map 1D turn_token tensor {-1.0, 0.0, +1.0} to branch_id {0, 1, 2}."""
        return torch.where(
            tokens_1d < -0.5,
            torch.zeros_like(tokens_1d, dtype=torch.long),
            torch.where(
                tokens_1d > 0.5,
                torch.full_like(tokens_1d, 2, dtype=torch.long),
                torch.ones_like(tokens_1d, dtype=torch.long),
            ),
        )

    def set_randomization(
        self,
        enabled: bool = True,
        allowed_tokens: list[float] | torch.Tensor | None = None,
    ) -> None:
        """Enable or disable multi-branch turn-token randomization and configure curriculum token pool."""
        self.randomize_turn_tokens = bool(enabled)
        self.multi_branch_enabled = bool(enabled)
        if allowed_tokens is not None:
            if isinstance(allowed_tokens, torch.Tensor):
                self.allowed_tokens = allowed_tokens.to(
                    device=self.device, dtype=torch.float32
                ).view(-1)
            else:
                self.allowed_tokens = torch.tensor(
                    list(allowed_tokens), device=self.device, dtype=torch.float32
                ).view(-1)
        all_mask = torch.ones(self.num_envs, dtype=torch.bool, device=self.device)
        if enabled:
            self.randomize_missions(all_mask)
        else:
            self.mission_tokens[:] = -1.0
            self.active_branch_ids[:] = 0
            self.fsm_state[:] = self.CRUISING
            self._branch_latched[:] = False
            self._in_inter_prev[:] = False
            self.exit_correct[:] = 0.0
            self.just_exited[:] = False

    def set_mission_tokens(
        self,
        tokens: torch.Tensor | float | int,
        env_ids: torch.Tensor | None = None,
        latch_branch_immediately: bool = True,
    ) -> None:
        """Assign explicit intersection turn tokens {-1.0, 0.0, +1.0} and enable multi-branch routing."""
        self.multi_branch_enabled = True
        if not isinstance(tokens, torch.Tensor):
            t_val = float(tokens)
            t_tensor = torch.full(
                (self.num_envs, self.num_agents),
                t_val,
                device=self.device,
                dtype=torch.float32,
            )
        else:
            t_tensor = tokens.to(device=self.device, dtype=torch.float32)
            if t_tensor.ndim == 0:
                t_tensor = t_tensor.view(1, 1).expand(self.num_envs, self.num_agents)
            elif t_tensor.ndim == 1:
                t_tensor = t_tensor.unsqueeze(1).expand(-1, self.num_agents)

        if env_ids is None:
            self.mission_tokens = t_tensor.clone()
            if latch_branch_immediately:
                self.active_branch_ids = self._tokens_to_branch_ids(
                    self.mission_tokens[:, 0]
                )
                self._branch_latched[:] = True
        else:
            if t_tensor.shape[0] == self.num_envs:
                self.mission_tokens[env_ids] = t_tensor[env_ids]
            else:
                self.mission_tokens[env_ids] = t_tensor
            if latch_branch_immediately:
                self.active_branch_ids[env_ids] = self._tokens_to_branch_ids(
                    self.mission_tokens[env_ids, 0]
                )
                self._branch_latched[env_ids] = True

    def is_multi_branch_active(self) -> bool:
        """Return True if multi-branch intersection routing or turn-token randomization is enabled."""
        env_flag = os.environ.get("ARCPRO_RANDOMIZE_TURN_TOKENS", "0").strip() in (
            "1",
            "true",
            "True",
        ) or os.environ.get("ARCPRO_MULTI_BRANCH", "0").strip() in (
            "1",
            "true",
            "True",
        )
        return bool(self.multi_branch_enabled or self.randomize_turn_tokens or env_flag)

    def get_active_branch_ids(self) -> torch.Tensor:
        """Return per-environment active branch IDs (0=Left, 1=Straight, 2=Right) of shape (num_envs,)."""
        return self.active_branch_ids

    def randomize_missions(self, env_mask: torch.Tensor):
        """Initialize or randomize per-environment intersection mission tokens.

        In default M005 single-loop mode (`randomize_turn_tokens=False`), initializes
        `turn_tokens` to 0.0 (lane follow) and `mission_tokens` to -1.0 (left loop).
        When `randomize_turn_tokens=True`, samples uniformly from `self.allowed_tokens`
        `{-1.0, 0.0, +1.0}` (or respects `ARCPRO_FORCE_TURN_TOKEN` if set).
        """
        mask_bool = env_mask.to(device=self.device, dtype=torch.bool).view(-1)
        mask_expanded = mask_bool.unsqueeze(1).expand(-1, self.num_agents)
        self.turn_tokens = torch.where(
            mask_expanded, torch.zeros_like(self.turn_tokens), self.turn_tokens
        )

        forced_env = os.environ.get("ARCPRO_FORCE_TURN_TOKEN", None)
        if forced_env is not None and forced_env.strip() != "":
            try:
                forced_val = float(forced_env.strip())
                self.multi_branch_enabled = True
                sampled = torch.full_like(self.mission_tokens, forced_val)
            except ValueError:
                sampled = torch.full_like(self.mission_tokens, -1.0)
        elif self.randomize_turn_tokens or (
            os.environ.get("ARCPRO_RANDOMIZE_TURN_TOKENS", "0").strip()
            in ("1", "true", "True")
        ):
            self.randomize_turn_tokens = True
            self.multi_branch_enabled = True
            n_pool = int(self.allowed_tokens.numel())
            rand_idx = torch.randint(
                0,
                max(1, n_pool),
                (self.num_envs, self.num_agents),
                device=self.device,
            )
            sampled = self.allowed_tokens[rand_idx]
        else:
            sampled = torch.full_like(self.mission_tokens, -1.0)

        self.mission_tokens = torch.where(mask_expanded, sampled, self.mission_tokens)
        if self.is_multi_branch_active():
            new_b = self._tokens_to_branch_ids(self.mission_tokens[:, 0])
            self.active_branch_ids = torch.where(
                mask_bool, new_b, self.active_branch_ids
            )
        else:
            self.active_branch_ids = torch.where(
                mask_bool,
                torch.zeros_like(self.active_branch_ids),
                self.active_branch_ids,
            )

    def reset(self, env_ids: torch.Tensor | None = None) -> None:
        """Reset per-environment intersection FSM state and re-sample mission tokens on episode reset."""
        if env_ids is None:
            mask = torch.ones(self.num_envs, dtype=torch.bool, device=self.device)
        else:
            mask = torch.zeros(self.num_envs, dtype=torch.bool, device=self.device)
            mask[env_ids] = True

        self.fsm_state = torch.where(
            mask, torch.zeros_like(self.fsm_state), self.fsm_state
        )
        self.exit_correct = torch.where(
            mask, torch.zeros_like(self.exit_correct), self.exit_correct
        )
        self.just_exited = torch.where(
            mask, torch.zeros_like(self.just_exited), self.just_exited
        )
        self._in_inter_prev = torch.where(
            mask, torch.zeros_like(self._in_inter_prev), self._in_inter_prev
        )
        self._branch_latched = torch.where(
            mask, torch.zeros_like(self._branch_latched), self._branch_latched
        )
        self.randomize_missions(mask)

    def update(self, env):
        """Update internal navigation state and active branch routing based on track topology zones."""
        if not self.initialized:
            self._initialize_gates()

        try:
            robot = env.scene["robot"]
            env_origins = env.scene.env_origins
            local_pos = robot.data.root_pos_w[:, :2] - env_origins[:, :2]

            if not self.is_multi_branch_active():
                tokens = torch.zeros(
                    (self.num_envs, self.num_agents), device=self.device
                )
                is_turning = in_any_corner(local_pos[:, 0:1], local_pos[:, 1:2])
                tokens = torch.where(
                    is_turning, torch.full_like(tokens, -1.0), tokens
                )
                self.turn_tokens = tokens
                self.active_branch_ids[:] = 0
                self.just_exited[:] = False
            else:
                x = local_pos[:, 0]
                y = local_pos[:, 1]
                in_inter = in_multi_branch_intersection(x, y)
                in_non_inter_corner = in_bend_turn(x, y) & (~in_inter)

                entering = in_inter & (~self._in_inter_prev)
                exiting = (~in_inter) & self._in_inter_prev

                # Transition CRUISING -> COMMITTED on intersection entry and latch branch
                self.fsm_state = torch.where(
                    entering,
                    torch.full_like(self.fsm_state, self.COMMITTED),
                    self.fsm_state,
                )
                mission_b = self._tokens_to_branch_ids(self.mission_tokens[:, 0])
                self.active_branch_ids = torch.where(
                    entering | in_inter | self._branch_latched,
                    mission_b,
                    self.active_branch_ids,
                )
                self._branch_latched = self._branch_latched | entering | in_inter

                # Evaluate exit completion on intersection exit
                self.just_exited = exiting
                if exiting.any():
                    self.fsm_state = torch.where(
                        exiting,
                        torch.full_like(self.fsm_state, self.CRUISING),
                        self.fsm_state,
                    )
                    self.exit_correct = torch.where(
                        exiting,
                        torch.ones_like(self.exit_correct),
                        self.exit_correct,
                    )

                # Emit turn_token: mission_tokens inside intersection zones, -1.0 in non-intersection bend, 0.0 on straights
                inter_expanded = in_inter.unsqueeze(1).expand(-1, self.num_agents)
                bend_expanded = in_non_inter_corner.unsqueeze(1).expand(
                    -1, self.num_agents
                )
                tokens = torch.where(
                    inter_expanded,
                    self.mission_tokens,
                    torch.where(
                        bend_expanded,
                        torch.full_like(self.mission_tokens, -1.0),
                        torch.zeros_like(self.mission_tokens),
                    ),
                )
                self.turn_tokens = tokens
                self._in_inter_prev = in_inter.clone()
        except Exception:
            self.turn_tokens = torch.zeros(
                (self.num_envs, self.num_agents), device=self.device
            )

        self.go_signals[:] = 1.0

    def get_nav_commands(self):
        """Returns (turn_tokens, go_signals) as tensors of shape (B, N)."""
        return self.turn_tokens, self.go_signals


_ROAD_MANAGER = None


def get_road_manager(num_envs: int = 1, num_agents: int = 1, device: str = "cuda:0"):
    global _ROAD_MANAGER
    if (
        _ROAD_MANAGER is None
        or _ROAD_MANAGER.num_envs != num_envs
        or _ROAD_MANAGER.num_agents != num_agents
        or str(_ROAD_MANAGER.device) != str(device)
    ):
        _ROAD_MANAGER = RoadManager(num_envs, num_agents, device=device)
    return _ROAD_MANAGER


import sys as _sys  # noqa: E402

_sys.modules.setdefault("mdp.road_manager", _sys.modules[__name__])
_sys.modules.setdefault("arcproLab.mdp.road_manager", _sys.modules[__name__])

