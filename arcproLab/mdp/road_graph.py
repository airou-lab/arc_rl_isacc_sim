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
        self.current_mission = "loop" # Default mission
        
        # Turn Tokens: -1.0 = LEFT, 0.0 = STRAIGHT, 1.0 = RIGHT
        self.turn_token = torch.zeros(1, device=self.device) # Batch size 1 by default
        self.go_signal = torch.ones(1, device=self.device)
        
        # Gate triggers (maps gate index to a decision)
        # This is a placeholder for a real graph-search result
        self.gate_decisions = {} 

    def update(self, env):
        """
        Updates the turn_token based on the robot's proximity to intersection gates.
        """
        from mdp.track_manager import get_track_manager
        tm = get_track_manager(device=self.device)
        
        # Get robot position
        asset = env.scene["robot"]
        pos_w = asset.data.root_pos_w - env.scene.env_origins
        
        # Ensure we have enough tokens for the number of envs
        if self.turn_token.shape[0] != env.num_envs:
            self.turn_token = torch.zeros(env.num_envs, device=self.device)
            self.go_signal = torch.ones(env.num_envs, device=self.device)

        # Logic: If we are within 2.0m of a Gate (Stop Line), look up the mission decision
        # For Phase 11, we default to STRAIGHT (0.0)
        # In a real scenario, this would check which gate we are at and set the token.
        
        # TODO: Implement trigger-based decision logic
        # For now, we stay STRAIGHT as per Phase 11 research recommendation
        self.turn_token[:] = 0.0
        self.go_signal[:] = 1.0

    def get_nav_commands(self):
        return self.turn_token, self.go_signal

_ROAD_GRAPH = None
def get_road_graph(device: str = "cuda:0"):
    global _ROAD_GRAPH
    if _ROAD_GRAPH is None:
        _ROAD_GRAPH = RoadGraph(device=device)
    return _ROAD_GRAPH
