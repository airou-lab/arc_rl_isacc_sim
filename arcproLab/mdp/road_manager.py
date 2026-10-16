# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import torch


class RoadManager:
    """
    Vectorized Road Manager for Multi-Agent Navigation.
    Replaces the singleton RoadGraph to support independent agent state.

    State Shape: (num_envs, num_agents)
    """

    def __init__(self, num_envs: int, num_agents: int, device: str = "cuda:0"):
        self.device = device
        self.num_envs = num_envs
        self.num_agents = num_agents

        # Turn Tokens: -1.0 = LEFT, 0.0 = STRAIGHT, 1.0 = RIGHT
        self.turn_tokens = torch.zeros((num_envs, num_agents), device=self.device)
        self.go_signals = torch.ones((num_envs, num_agents), device=self.device)

        # Random initial missions
        self.randomize_missions(
            torch.ones(num_envs, dtype=torch.bool, device=self.device)
        )

        # Gate discovery
        self.gates_pos = None
        self.gates_intent = None
        self.initialized = False

    def _initialize_gates(self):
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

    def randomize_missions(self, env_mask: torch.Tensor):
        """Initializes turn tokens to 0.0 (lane follow) by default, avoiding random wall-crash commands."""
        mask_expanded = env_mask.unsqueeze(1).expand(-1, self.num_agents)
        self.turn_tokens = torch.where(
            mask_expanded, torch.zeros_like(self.turn_tokens), self.turn_tokens
        )

    def update(self, env):
        """Updates internal state based on environment triggers and vehicle proximity to gates."""
        if not self.initialized:
            self._initialize_gates()

        # Proximity-based dynamic gate intent routing:
        # Default to 0.0 (Straight / Lane Follow) on straightaways and non-gate zones
        try:
            robot = env.scene["robot"]
            env_origins = env.scene.env_origins
            local_pos = robot.data.root_pos_w[:, :2] - env_origins[:, :2]

            tokens = torch.zeros((self.num_envs, self.num_agents), device=self.device)

            # Circuit Turn Token Routing (Counter-Clockwise Circuit, all turns are LEFT):
            # 1. Corner 1 Turn (Leg 1 South -> Leg 2 East): curve approach begins at Y < -5.60, completes at X >= -15.00
            in_c1_turn = (local_pos[:, 0:1] < -15.00) & (local_pos[:, 1:2] < -5.60)
            # 2. Corner 2 / Intersection Turn (Leg 2 East -> North Straight):
            # Curve begins at X > -6.80, completes when vehicle enters North straight at Y >= -3.60
            in_c2_turn = (
                (local_pos[:, 0:1] > -6.80)
                & (local_pos[:, 0:1] < -2.0)
                & (local_pos[:, 1:2] >= -7.5)
                & (local_pos[:, 1:2] <= -3.60)
            )
            # 3. Sweeping Road Bend Approach (North Straight -> Top Straight West):
            # Curve begins at Y >= 2.20, completes at X <= -7.80, Y >= 6.80
            in_bend_turn = (
                (local_pos[:, 1:2] >= 2.20)
                & (local_pos[:, 1:2] <= 7.20)
                & (local_pos[:, 0:1] > -7.80)
                & (local_pos[:, 0:1] < -2.0)
            )
            # 4. Corner 4 Turn (Top Straight West -> Leg 1 South): curve approach begins at X < -14.50, completes at Y <= 5.50
            in_c4_turn = (local_pos[:, 0:1] < -14.50) & (local_pos[:, 1:2] > 5.50)

            is_turning = in_c1_turn | in_c2_turn | in_bend_turn | in_c4_turn
            tokens = torch.where(
                is_turning, torch.tensor(-1.0, device=self.device), tokens
            )

            self.turn_tokens = tokens
        except Exception:
            self.turn_tokens = torch.zeros(
                (self.num_envs, self.num_agents), device=self.device
            )

        # Future: FIFO Queue logic for intersections (Milestone 4)
        self.go_signals[:] = 1.0

    def get_nav_commands(self):
        """Returns (turn_tokens, go_signals) as tensors of shape (B, N)."""
        return self.turn_tokens, self.go_signals


_ROAD_MANAGER = None


def get_road_manager(num_envs: int = 1, num_agents: int = 1, device: str = "cuda:0"):
    global _ROAD_MANAGER
    if _ROAD_MANAGER is None:
        _ROAD_MANAGER = RoadManager(num_envs, num_agents, device=device)
    return _ROAD_MANAGER
