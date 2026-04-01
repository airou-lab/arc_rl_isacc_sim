# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import torch
import numpy as np
import os

class TrackManager:
    """
    Manages track waypoints and provides vectorized distance/heading queries.
    Used by ObservationManager to compute lateral and heading errors.
    """
    def __init__(self, device: str = "cuda:0"):
        self.device = device
        self.waypoints = None # (N, 3) - [x, y, yaw]
        
        # Default path to waypoints file
        self.wp_path = os.path.join(os.path.dirname(__file__), "track_centerline.npy")
        
        if os.path.exists(self.wp_path):
            self.load_waypoints(self.wp_path)
        else:
            # Try to sample from USD if we are in a running simulation
            try:
                print(f"[TrackManager] {self.wp_path} not found. Attempting to sample from USD...")
                self.sample_waypoints_from_usd()
                if self.waypoints is not None:
                    self.save_waypoints(self.wp_path)
            except Exception as e:
                # Fallback: Create a simple straight line for Road A if file missing and USD sampling fails
                # Starting at (-125, 62) and going North (+Y)
                print(f"[TrackManager] Warning: USD sampling failed ({e}). Using fallback straight line.")
                y_coords = np.linspace(62, 200, 100)
                wps = np.zeros((100, 3))
                wps[:, 0] = -125.0 # X
                wps[:, 1] = y_coords # Y
                wps[:, 2] = np.pi / 2.0 # Yaw (North)
                self.waypoints = torch.tensor(wps, device=self.device, dtype=torch.float32)

    def load_waypoints(self, path: str):
        """Loads waypoints from a .npy file."""
        data = np.load(path)
        self.waypoints = torch.tensor(data, device=self.device, dtype=torch.float32)
        print(f"[TrackManager] Loaded {len(self.waypoints)} waypoints from {path}")

    def save_waypoints(self, path: str):
        """Saves waypoints to a .npy file."""
        if self.waypoints is not None:
            data = self.waypoints.cpu().numpy()
            np.save(path, data)
            print(f"[TrackManager] Saved {len(self.waypoints)} waypoints to {path}")

    def sample_waypoints_from_usd(self, density: float = 0.5):
        """
        Scans the USD stage for road meshes and generates centerline waypoints.
        Args:
            density: Desired distance between waypoints in meters.
        """
        import omni.usd
        from pxr import Usd, UsdGeom, Gf
        
        stage = omni.usd.get_context().get_stage()
        if stage is None:
            raise RuntimeError("USD stage not found. USD sampling requires a running simulation.")

        print(f"[TrackManager] Scanning stage: {stage.GetRootLayer().identifier}")
        raw_points = []
        # Traverse the stage using PrimRange for better reference handling
        for prim in Usd.PrimRange(stage.GetPseudoRoot()):
            if prim.IsA(UsdGeom.Mesh):
                prim_path = str(prim.GetPath())
                # Filter for track meshes. 
                # In arcpro_RL_open_street_sim.usd, road meshes are under /World/drivable_surfaces
                is_track = any(keyword in prim_path.lower() for keyword in ["pavement", "road", "track", "drivable_surfaces"])
                
                if is_track:
                    mesh = UsdGeom.Mesh(prim)
                    points = mesh.GetPointsAttr().Get()
                    if points:
                        # Get world transform
                        xform = UsdGeom.Xformable(prim)
                        world_transform = xform.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
                        print(f" - Found track mesh: {prim_path} ({len(points)} points)")
                        for p in points:
                            p_world = world_transform.Transform(p)
                            raw_points.append([p_world[0], p_world[1]])

        if not raw_points:
            # Fallback: List all meshes to see what we missed
            all_meshes = [str(p.GetPath()) for p in Usd.PrimRange(stage.GetPseudoRoot()) if p.IsA(UsdGeom.Mesh)]
            print(f"[TrackManager] ERR: No road meshes found. Total meshes in stage: {len(all_meshes)}")
            if all_meshes:
                print(f" - Example mesh paths: {all_meshes[:5]}")
            raise RuntimeError("No road meshes found in USD stage.")

        print(f"[TrackManager] Collected {len(raw_points)} raw points from USD.")
        pts = np.array(raw_points)
        # Round to 2 decimal places for deduplication
        pts = np.unique(np.round(pts, 2), axis=0)

        # 2. Simple ordering (Nearest Neighbor)
        ordered_pts = []
        curr_idx = 0
        visited = set([0])
        ordered_pts.append(pts[0])
        
        while len(visited) < len(pts):
            last_pt = ordered_pts[-1]
            dists = np.sum((pts - last_pt)**2, axis=1)
            # Mask visited
            for v in visited:
                dists[v] = np.inf
            
            next_idx = np.argmin(dists)
            if dists[next_idx] > 5.0: # Break if gap is too large (likely separate track segments)
                break
            
            ordered_pts.append(pts[next_idx])
            visited.add(next_idx)
            curr_idx = next_idx
            
        ordered_pts = np.array(ordered_pts)

        # 3. Resample to fixed density
        # Calculate cumulative distance
        diffs = np.diff(ordered_pts, axis=0)
        segment_dists = np.sqrt(np.sum(diffs**2, axis=1))
        cum_dist = np.concatenate(([0], np.cumsum(segment_dists)))
        
        total_dist = cum_dist[-1]
        num_samples = int(total_dist / density)
        interp_dists = np.linspace(0, total_dist, num_samples)
        
        resampled_x = np.interp(interp_dists, cum_dist, ordered_pts[:, 0])
        resampled_y = np.interp(interp_dists, cum_dist, ordered_pts[:, 1])
        
        # 4. Compute Tangents/Yaw
        dx = np.gradient(resampled_x)
        dy = np.gradient(resampled_y)
        yaws = np.arctan2(dy, dx)
        
        wps = np.stack([resampled_x, resampled_y, yaws], axis=1)
        
        # 5. Shift to Origin (match Isaac Lab env spawn at 0,0)
        offset = wps[0, :2].copy()
        wps[:, :2] -= offset
        print(f"[TrackManager] Shifted waypoints by {offset} to center at (0,0)")
        
        self.waypoints = torch.tensor(wps, device=self.device, dtype=torch.float32)
        print(f"[TrackManager] Successfully sampled {len(self.waypoints)} waypoints from USD.")

    def get_closest_waypoint_data(self, pos: torch.Tensor) -> torch.Tensor:
        """
        Finds the closest waypoint for each environment.
        Args:
            pos: (num_envs, 3) - World positions of robots
        Returns:
            (num_envs, 3) - [wp_x, wp_y, wp_yaw] of closest waypoints
        """
        # (num_envs, 1, 2) - robots
        # (1, num_waypoints, 2) - waypoints
        diff = pos[:, None, :2] - self.waypoints[None, :, :2]
        dist_sq = torch.sum(diff**2, dim=-1)
        closest_indices = torch.argmin(dist_sq, dim=-1)
        return self.waypoints[closest_indices]

    def compute_errors(self, pos: torch.Tensor, yaw: torch.Tensor):
        """
        Computes lateral and heading errors relative to the track centerline.
        Args:
            pos: (num_envs, 3) - World positions
            yaw: (num_envs,) - Current yaw angles
        Returns:
            lat_err: (num_envs,)
            head_err: (num_envs,)
        """
        wp_data = self.get_closest_waypoint_data(pos)
        wp_pos = wp_data[:, :2]
        wp_yaw = wp_data[:, 2]
        
        # Heading error: robot_yaw - track_yaw
        head_err = yaw - wp_yaw
        # Wrap to [-pi, pi]
        head_err = torch.atan2(torch.sin(head_err), torch.cos(head_err))
        
        # Lateral error: signed distance to the track centerline
        # vector from wp to robot
        v_wp_to_rob = pos[:, :2] - wp_pos
        # track tangent vector (cos(yaw), sin(yaw))
        v_tangent = torch.stack([torch.cos(wp_yaw), torch.sin(wp_yaw)], dim=-1)
        # track normal vector (-sin(yaw), cos(yaw))
        v_normal = torch.stack([-torch.sin(wp_yaw), torch.cos(wp_yaw)], dim=-1)
        
        # Signed distance = dot product with normal
        lat_err = torch.sum(v_wp_to_rob * v_normal, dim=-1)
        
        return lat_err, head_err

# Singleton instance for the environment
_TRACK_MANAGER = None

def get_track_manager(device: str = "cuda:0"):
    global _TRACK_MANAGER
    if _TRACK_MANAGER is None:
        _TRACK_MANAGER = TrackManager(device=device)
    return _TRACK_MANAGER
