# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import torch
import numpy as np

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
        self.randomize_missions(torch.ones(num_envs, dtype=torch.bool, device=self.device))
        
        # Gate discovery
        self.gates_pos = None
        self.gates_intent = None
        self.initialized = False

    def _initialize_gates(self):
        import omni.usd
        from pxr import Usd, UsdGeom
        stage = omni.usd.get_context().get_stage()
        if not stage: return
        
        gate_positions = []
        gate_intents = []
        
        for prim in stage.Traverse():
            if "laneGate" in prim.GetName():
                xform = UsdGeom.Xformable(prim)
                world_transform = xform.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
                pos = world_transform.ExtractTranslation()
                
                intent = 0.0
                attr = prim.GetAttribute("SignalTurnRelation")
                if attr.IsValid():
                    val = attr.Get()
                    if isinstance(val, str):
                        if val == "Left": intent = -1.0
                        elif val == "Right": intent = 1.0
                        else: intent = 0.0
                    else:
                        intent = float(val)
                
                gate_positions.append([pos[0], pos[1]])
                gate_intents.append(intent)
        
        if gate_positions:
            self.gates_pos = torch.tensor(gate_positions, device=self.device, dtype=torch.float32)
            self.gates_intent = torch.tensor(gate_intents, device=self.device, dtype=torch.float32)
            print(f"[RoadManager] Discovered {len(gate_positions)} navigation gates.")
        
        self.initialized = True

    def randomize_missions(self, env_mask: torch.Tensor):
        """Randomizes the turn intent for all agents in the masked environments."""
        new_tokens = torch.randint(-1, 2, (self.num_envs, self.num_agents), device=self.device).float()
        # Expand mask to (B, N)
        mask_expanded = env_mask.unsqueeze(1).expand(-1, self.num_agents)
        self.turn_tokens = torch.where(mask_expanded, new_tokens, self.turn_tokens)

    def update(self, env):
        """Updates internal state based on environment triggers."""
        if not self.initialized:
            self._initialize_gates()

        # Update missions on reset
        reset_buf = getattr(env, "reset_buf", None)
        if reset_buf is not None and reset_buf.any():
            self.randomize_missions(reset_buf)
        
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
