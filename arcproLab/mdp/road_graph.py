# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import torch
import numpy as np

class RoadGraph:
    """
    Manages high-level navigation decisions and Turn Token logic.
    Provides the 'Intent' (Left/Straight/Right) based on a mission or route.
    """
    def __init__(self, device: str = "cuda:0"):
        self.device = device
        
        # Turn Tokens: -1.0 = LEFT, 0.0 = STRAIGHT, 1.0 = RIGHT
        self.turn_token = None
        self.go_signal = None
        
        # Gate discovery (Lazy initialization in update)
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
                # Get position
                xform = UsdGeom.Xformable(prim)
                world_transform = xform.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
                pos = world_transform.ExtractTranslation()
                
                # Get intent from attribute (Default to Straight if missing)
                # Note: SignalTurnRelation: -1=Left, 0=Straight, 1=Right
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
            print(f"[RoadGraph] Discovered {len(gate_positions)} navigation gates.")
        
        self.initialized = True

    def update(self, env):
        """
        Updates the turn_token based on the robot's proximity to intersection gates.
        """
        if not self.initialized:
            self._initialize_gates()

        # Initialize tensors for the current number of environments
        if self.turn_token is None or self.turn_token.shape[0] != env.num_envs:
            self.turn_token = torch.zeros(env.num_envs, device=env.device)
            self.go_signal = torch.ones(env.num_envs, device=env.device)
            # Initial randomization
            self.turn_token = torch.randint(-1, 2, (env.num_envs,), device=self.device).float()

        # Logic: Randomize mission ONLY on environment reset
        # This keeps the 'Intent' persistent so the robot can learn to follow it
        reset_buf = getattr(env, "reset_buf", None)
        if reset_buf is not None and reset_buf.any():
            new_tokens = torch.randint(-1, 2, (env.num_envs,), device=self.device).float()
            self.turn_token = torch.where(reset_buf, new_tokens, self.turn_token)
        
        self.go_signal[:] = 1.0

    def get_nav_commands(self):
        return self.turn_token, self.go_signal

_ROAD_GRAPH = None
def get_road_graph(device: str = "cuda:0"):
    global _ROAD_GRAPH
    if _ROAD_GRAPH is None:
        _ROAD_GRAPH = RoadGraph(device=device)
    return _ROAD_GRAPH
