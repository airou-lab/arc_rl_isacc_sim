# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import torch
import numpy as np
import time

class V2VManager:
    """
    Simulates Zero-Localization V2V Queueing for Smart Intersections.
    
    Logic:
    1. Robots broadcast 'Stopped' when near an arrival gate and speed < threshold.
    2. Joining the FCFS queue for that intersection.
    3. 'Go Signal' is granted only if the robot is at the head of the queue.
    4. Robots broadcast 'Passed' when near an exit gate to clear the queue.
    """
    def __init__(self, num_envs: int, num_agents: int, device: str = "cuda:0"):
        self.num_envs = num_envs
        self.num_agents = num_agents
        self.device = device
        
        # Queues: {env_id -> [agent_index_0, agent_index_1, ...]}
        self.queues = [[] for _ in range(num_envs)]
        
        # State tracking
        self.agent_status = torch.zeros((num_envs, num_agents), device=device) # 0=Idle, 1=InQueue, 2=Clearing
        
        # Gate positions (Cached from TrackManager or RoadGraph)
        self.arrival_gates = None
        self.exit_gates = None
        
        self.initialized = False

    def _initialize_gates(self, env):
        # In a real setup, we'd use the USD prims. 
        # For the prototype, we assume the intersection center is (-16.25, 0.35)
        # and arrival gates are ~2.5m out from center.
        cx, cy = -16.25, 0.35
        dist = 2.5
        
        # North, South, East, West arrival points
        self.arrival_gates = torch.tensor([
            [cx, cy + dist], # North
            [cx, cy - dist], # South
            [cx + dist, cy], # East
            [cx - dist, cy], # West
        ], device=self.device)
        
        # Exit gates are on the opposite side of the intersection or beyond the center
        self.exit_gates = torch.tensor([
            [cx, cy - 1.0], # South Exit
            [cx, cy + 1.0], # North Exit
            [cx - 1.0, cy], # West Exit
            [cx + 1.0, cy], # East Exit
        ], device=self.device)
        
        self.initialized = True

    def update(self, env):
        if not self.initialized:
            self._initialize_gates(env)
            
        # Reset queues for environments that just reset
        reset_buf = getattr(env, "reset_buf", None)
        if reset_buf is not None:
            for env_id in torch.where(reset_buf)[0]:
                self.queues[env_id.item()] = []
                self.agent_status[env_id] = 0

        # Update each agent's status based on proximity to gates and speed
        for i in range(self.num_agents):
            robot = env.scene[f"robot_{i}"]
            pos = robot.data.root_pos_w[:, :2] - env.scene.env_origins[:, :2]
            vel = robot.data.root_lin_vel_b[:, 0]
            
            for env_id in range(self.num_envs):
                status = self.agent_status[env_id, i]
                
                # 1. Check for Arrival (Join Queue)
                # If near ANY arrival gate AND slow AND not already in queue
                dist_to_arrival = torch.norm(pos[env_id] - self.arrival_gates, dim=1).min()
                if status == 0 and dist_to_arrival < 0.5 and abs(vel[env_id]) < 0.1:
                    if i not in self.queues[env_id]:
                        self.queues[env_id].append(i)
                        self.agent_status[env_id, i] = 1
                        print(f"[V2V] Env {env_id}: Robot_{i} joined the queue.")

                # 2. Check for Exit (Clear Queue)
                # If in queue (or clearing) AND near ANY exit gate
                dist_to_exit = torch.norm(pos[env_id] - self.exit_gates, dim=1).min()
                if status >= 1 and dist_to_exit < 0.8:
                    if i in self.queues[env_id]:
                        self.queues[env_id].remove(i)
                        self.agent_status[env_id, i] = 0
                        print(f"[V2V] Env {env_id}: Robot_{i} cleared the intersection.")

    def get_go_signal(self, env_id: int, agent_index: int) -> float:
        """Returns 1.0 if the agent is at the head of the queue or not in a queue."""
        queue = self.queues[env_id]
        
        # If not in queue, you have GO (to approach the stop line)
        if agent_index not in queue:
            return 1.0
            
        # If at the head of the queue, you have GO (to cross the intersection)
        if queue[0] == agent_index:
            return 1.0
            
        # Otherwise, WAIT
        return 0.0

_V2V_MANAGER = None
def get_v2v_manager(num_envs: int, num_agents: int, device: str = "cuda:0"):
    global _V2V_MANAGER
    if _V2V_MANAGER is None:
        _V2V_MANAGER = V2VManager(num_envs, num_agents, device)
    return _V2V_MANAGER
