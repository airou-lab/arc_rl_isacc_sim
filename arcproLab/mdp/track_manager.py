# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import torch
import numpy as np
import os
from .road_graph import RoadGraph

class TrackManager:
    """
    Manages track waypoints and provides vectorized distance/heading queries.
    Supports multi-segment navigation via RoadGraph.
    """
    def __init__(self, device: str = "cuda:0", num_envs: int = 1):
        self.device = device
        self.num_envs = num_envs
        self.graph = RoadGraph()
        self.waypoints = None # Legacy single-list support
        
        # Track current segment per environment: (node_u, node_v)
        self.current_edges = [None] * num_envs
        
        # Default path to waypoints file
        self.wp_path = os.path.join(os.path.dirname(__file__), "track_centerline.npy")
        
        if os.path.exists(self.wp_path):
            self.load_waypoints(self.wp_path)
            # Initialize graph with a single edge for backward compatibility
            self._initialize_single_segment_graph()
        else:
            # Try to sample from USD if we are in a running simulation
            try:
                print(f"[TrackManager] {self.wp_path} not found. Attempting to sample from USD...")
                self.sample_waypoints_from_usd()
                if self.waypoints is not None:
                    self.save_waypoints(self.wp_path)
                    self._initialize_single_segment_graph()
            except Exception as e:
                print(f"[TrackManager] Warning: USD sampling failed ({e}). Using fallback.")
                self._load_fallback_line()
                self._initialize_single_segment_graph()

    def _initialize_single_segment_graph(self):
        """Creates a simple 2-node graph for a single track loop/segment."""
        if self.waypoints is None: return
        wps_np = self.waypoints.cpu().numpy()
        self.graph.add_node(0, wps_np[0, :2])
        self.graph.add_node(1, wps_np[-1, :2])
        self.graph.add_edge(0, 1, wps_np)
        for i in range(self.num_envs):
            self.current_edges[i] = (0, 1)

    def _load_fallback_line(self):
        y_coords = np.linspace(62, 200, 100)
        wps = np.zeros((100, 3))
        wps[:, 0] = -125.0 # X
        wps[:, 1] = y_coords # Y
        wps[:, 2] = np.pi / 2.0 # Yaw (North)
        self.waypoints = torch.tensor(wps, device=self.device, dtype=torch.float32)

    def load_waypoints(self, path: str):
        """Loads waypoints from a .npy file and partitions them into graph edges."""
        data = np.load(path)
        self.waypoints = torch.tensor(data, device=self.device, dtype=torch.float32)
        print(f"[TrackManager] Loaded {len(self.waypoints)} waypoints from {path}")
        
        # Partition into segments if possible
        # Logic: If distance between consecutive points > 2.0m, it's a new segment
        diffs = np.diff(data[:, :2], axis=0)
        dists = np.sqrt(np.sum(diffs**2, axis=1))
        gaps = np.where(dists > 2.0)[0]
        
        start_idx = 0
        for gap_idx in gaps:
            segment = data[start_idx : gap_idx + 1]
            if len(segment) > 1:
                u = self.graph._find_or_create_node(segment[0, :2], 0.1)
                v = self.graph._find_or_create_node(segment[-1, :2], 0.1)
                self.graph.add_edge(u, v, segment)
            start_idx = gap_idx + 1
            
        # Last segment
        segment = data[start_idx:]
        if len(segment) > 1:
            u = self.graph._find_or_create_node(segment[0, :2], 0.1)
            v = self.graph._find_or_create_node(segment[-1, :2], 0.1)
            self.graph.add_edge(u, v, segment)
            
        print(f"[TrackManager] Partitioned waypoints into {len(self.graph.edges)} segments.")
        # Connect nodes that are close to each other to form a loop
        self._auto_connect_graph()
        
        # Initialize envs to first edge
        if self.graph.edges:
            first_edge = list(self.graph.edges.keys())[0]
            for i in range(self.num_envs):
                self.current_edges[i] = first_edge

    def _auto_connect_graph(self, threshold=0.5):
        """Connects leaf nodes to nearby nodes to ensure continuity."""
        for u in list(self.graph.nodes.keys()):
            if not self.graph.get_neighbors(u):
                # Leaf node, look for nearby nodes
                u_pos = self.graph.nodes[u]
                for v, v_pos in self.graph.nodes.items():
                    if u != v and np.linalg.norm(u_pos[:2] - v_pos[:2]) < threshold:
                        # Close enough to connect (Zero-length virtual edge)
                        # In a real track, we'd add a small segment or just merge nodes
                        # For now, merge node u into v in the adjacency
                        print(f"[TrackManager] Auto-connecting Node {u} -> {v} (loop closure)")
                        # Find edges pointing to u and point them to v instead
                        for edge, data in list(self.graph.edges.items()):
                            if edge[1] == u:
                                self.graph.edges.pop(edge)
                                self.graph.add_edge(edge[0], v, data["waypoints"])
                                if u in self.graph.adjacency[edge[0]]:
                                    self.graph.adjacency[edge[0]].remove(u)
                                    self.graph.adjacency[edge[0]].append(v)

    def save_waypoints(self, path: str):
        """Saves waypoints to a .npy file."""
        if self.waypoints is not None:
            data = self.waypoints.cpu().numpy()
            np.save(path, data)
            print(f"[TrackManager] Saved {len(self.waypoints)} waypoints to {path}")

    def sample_waypoints_from_usd(self, density: float = 0.5):
        """
        Scans the USD stage for road meshes and builds a multi-segment RoadGraph.
        """
        self.graph.sample_graph_from_usd(density=density)
        
        if not self.graph.edges:
            # Fallback to single list logic if graph build failed
            print("[TrackManager] Graph auto-discovery failed. Falling back to single-loop discovery.")
            self._sample_legacy_waypoints(density)
        else:
            print(f"[TrackManager] Auto-discovered graph with {len(self.graph.nodes)} nodes and {len(self.graph.edges)} edges.")
            # Pick a starting edge for all envs
            first_edge = list(self.graph.edges.keys())[0]
            for i in range(self.num_envs):
                self.current_edges[i] = first_edge
            
            # For legacy compatibility, concatenate all waypoints into self.waypoints
            all_wps = []
            for edge_data in self.graph.edges.values():
                all_wps.append(edge_data["waypoints"])
            self.waypoints = torch.tensor(np.concatenate(all_wps, axis=0), device=self.device, dtype=torch.float32)

    def _sample_legacy_waypoints(self, density):
        # ... (Implementation of the old nearest-neighbor loop logic)
        pass

    def get_closest_waypoint_data(self, pos: torch.Tensor) -> torch.Tensor:
        """
        Finds the closest waypoint for each environment on its current segment.
        Args:
            pos: (num_envs, 3) - World positions of robots
        Returns:
            (num_envs, 3) - [wp_x, wp_y, wp_yaw] of closest waypoints
        """
        results = torch.zeros((self.num_envs, 3), device=self.device)
        
        for i in range(self.num_envs):
            edge = self.current_edges[i]
            if edge is None:
                # Fallback to legacy behavior if no edge assigned
                diff = pos[i, :2] - self.waypoints[:, :2]
                dist_sq = torch.sum(diff**2, dim=-1)
                closest_idx = torch.argmin(dist_sq)
                results[i] = self.waypoints[closest_idx]
                continue
                
            edge_data = self.graph.get_edge_data(*edge)
            wps = torch.tensor(edge_data["waypoints"], device=self.device, dtype=torch.float32)
            
            diff = pos[i, :2] - wps[:, :2]
            dist_sq = torch.sum(diff**2, dim=-1)
            closest_idx = torch.argmin(dist_sq)
            results[i] = wps[closest_idx]
            
            # TODO: Transition logic: If we are close to the last waypoint of the edge, switch to next edge
            if closest_idx > len(wps) * 0.9:
                self.switch_segment(i)
                
        return results

    def switch_segment(self, env_id):
        """Transitions environment to the next segment in the graph."""
        curr_edge = self.current_edges[env_id]
        if curr_edge is None: return
        
        next_node = self.graph.find_next_edge(*curr_edge)
        if next_node is not None:
            self.current_edges[env_id] = (curr_edge[1], next_node)
            print(f"[TrackManager] Env {env_id} transitioned to edge {(curr_edge[1], next_node)}")

    def compute_errors(self, pos: torch.Tensor, yaw: torch.Tensor):
        """
        Computes lateral and heading errors relative to the current segment centerline.
        """
        wp_data = self.get_closest_waypoint_data(pos)
        wp_pos = wp_data[:, :2]
        wp_yaw = wp_data[:, 2]
        
        # Heading error: robot_yaw - track_yaw
        head_err = yaw - wp_yaw
        # Wrap to [-pi, pi]
        head_err = torch.atan2(torch.sin(head_err), torch.cos(head_err))
        
        # Lateral error: signed distance to the track centerline
        v_wp_to_rob = pos[:, :2] - wp_pos
        v_normal = torch.stack([-torch.sin(wp_yaw), torch.cos(wp_yaw)], dim=-1)
        
        lat_err = torch.sum(v_wp_to_rob * v_normal, dim=-1)
        
        return lat_err, head_err

# Singleton instance for the environment
_TRACK_MANAGER = None

def get_track_manager(device: str = "cuda:0", num_envs: int = 1):
    global _TRACK_MANAGER
    if _TRACK_MANAGER is None:
        _TRACK_MANAGER = TrackManager(device=device, num_envs=num_envs)
    return _TRACK_MANAGER
