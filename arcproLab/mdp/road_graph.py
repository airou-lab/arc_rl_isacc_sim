# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import numpy as np
import json
import os

class RoadGraph:
    """
    A graph-based representation of the road network.
    Nodes represent intersections or endpoints.
    Edges represent road segments containing waypoints.
    """
    def __init__(self):
        self.nodes = {} # id -> (x, y, z)
        self.edges = {} # (node_u, node_v) -> {waypoints: np.array, length: float}
        self.adjacency = {} # node_u -> list of node_v

    def add_node(self, node_id, coords):
        """Adds a node to the graph."""
        self.nodes[node_id] = np.array(coords)
        if node_id not in self.adjacency:
            self.adjacency[node_id] = []

    def add_edge(self, u, v, waypoints):
        """Adds a directed edge between nodes u and v with waypoints."""
        if u not in self.nodes or v not in self.nodes:
            raise ValueError(f"Nodes {u} or {v} not found in graph.")
        
        self.edges[(u, v)] = {
            "waypoints": np.array(waypoints),
            "length": self._calculate_length(waypoints)
        }
        
        if v not in self.adjacency[u]:
            self.adjacency[u].append(v)

    def get_neighbors(self, u):
        """Returns a list of neighbor node IDs for node u."""
        return self.adjacency.get(u, [])

    def get_edge_data(self, u, v):
        """Returns the data for edge (u, v)."""
        return self.edges.get((u, v))

    def find_next_edge(self, u, v):
        """
        Heuristic to find the 'next' edge after completing (u, v).
        Returns a neighbor node 'w' such that (v, w) is an edge.
        """
        neighbors = self.get_neighbors(v)
        if not neighbors:
            return None
        # Simple policy: pick first neighbor (can be extended for routing)
        return neighbors[0]

    def _calculate_length(self, waypoints):
        """Calculates the total length of a segment."""
        if len(waypoints) < 2:
            return 0.0
        diffs = np.diff(waypoints[:, :2], axis=0)
        dists = np.sqrt(np.sum(diffs**2, axis=1))
        return np.sum(dists)

    def save_to_json(self, path):
        """Saves the graph structure to a JSON file."""
        data = {
            "nodes": {str(k): v.tolist() for k, v in self.nodes.items()},
            "edges": []
        }
        for (u, v), edge_data in self.edges.items():
            data["edges"].append({
                "u": u,
                "v": v,
                "waypoints": edge_data["waypoints"].tolist()
            })
        
        with open(path, 'w') as f:
            json.dump(data, f, indent=4)

    def load_from_json(self, path):
        """Loads the graph structure from a JSON file."""
        if not os.path.exists(path):
            return
        
        with open(path, 'r') as f:
            data = json.load(f)
        
        self.nodes = {int(k): np.array(v) for k, v in data["nodes"].items()}
        self.edges = {}
        self.adjacency = {k: [] for k in self.nodes}
        
        for edge in data["edges"]:
            u, v = edge["u"], edge["v"]
            self.add_edge(u, v, edge["waypoints"])

    def sample_graph_from_usd(self, density: float = 0.5):
        """
        Scans the USD stage for road meshes and builds the graph automatically.
        Connects segments based on endpoint proximity.
        """
        import omni.usd
        from pxr import Usd, UsdGeom, Gf
        
        stage = omni.usd.get_context().get_stage()
        if stage is None:
            raise RuntimeError("USD stage not found. USD sampling requires a running simulation.")

        # 1. Identify road meshes and extract their points as segments
        segments = []
        for prim in Usd.PrimRange(stage.GetPseudoRoot()):
            if prim.IsA(UsdGeom.Mesh):
                prim_path = str(prim.GetPath())
                is_road = any(k in prim_path.lower() for k in ["road", "pavement", "drivable_surfaces"])
                if is_road and not any(k in prim_path.lower() for k in ["grass", "terrain"]):
                    mesh = UsdGeom.Mesh(prim)
                    points = mesh.GetPointsAttr().Get()
                    if points:
                        xform = UsdGeom.Xformable(prim)
                        world_transform = xform.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
                        seg_points = []
                        for p in points:
                            p_world = world_transform.Transform(p)
                            seg_points.append([p_world[0], p_world[1]])
                        
                        # Simplification: Sort points to create a line segment
                        pts = np.array(seg_points)
                        pts = np.unique(np.round(pts, 2), axis=0)
                        if len(pts) > 1:
                            segments.append(pts)

        if not segments:
            return

        # 2. Build nodes and edges based on segments
        # For now, treat each segment as an edge between its endpoints
        node_counter = 0
        for seg in segments:
            # endpoints
            p_start, p_end = seg[0], seg[-1]
            
            # Find or create nodes
            u = self._find_or_create_node(p_start, 0.1)
            v = self._find_or_create_node(p_end, 0.1)
            
            # Resample segment for waypoints
            resampled = self._resample_segment(seg, density)
            self.add_edge(u, v, resampled)

    def _find_or_create_node(self, coords, threshold):
        """Finds an existing node near coords or creates a new one."""
        for node_id, node_coords in self.nodes.items():
            if np.linalg.norm(node_coords[:2] - coords[:2]) < threshold:
                return node_id
        
        new_id = len(self.nodes)
        self.add_node(new_id, [coords[0], coords[1], 0.0])
        return new_id

    def _resample_segment(self, points, density):
        """Resamples points to a fixed density and calculates yaw."""
        # Cumulative distance
        diffs = np.diff(points, axis=0)
        segment_dists = np.sqrt(np.sum(diffs**2, axis=1))
        cum_dist = np.concatenate(([0], np.cumsum(segment_dists)))
        
        total_dist = cum_dist[-1]
        num_samples = max(2, int(total_dist / density))
        interp_dists = np.linspace(0, total_dist, num_samples)
        
        resampled_x = np.interp(interp_dists, cum_dist, points[:, 0])
        resampled_y = np.interp(interp_dists, cum_dist, points[:, 1])
        
        dx = np.gradient(resampled_x)
        dy = np.gradient(resampled_y)
        yaws = np.arctan2(dy, dx)
        
        return np.stack([resampled_x, resampled_y, yaws], axis=1)
