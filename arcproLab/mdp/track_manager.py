# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import torch
import numpy as np
import os

class TrackManager:
    """
    Manages track markers and provides direct distance-to-boundary queries.
    Ditches problematic centerline math for robust marker-based termination.
    """
    def __init__(self, device: str = "cuda:0"):
        self.device = device
        self.waypoints = None
        self.visualizer = None
        self.left_visualizer = None
        self.right_visualizer = None
        
        self.raw_yellow_pts = None # CPU numpy
        self.raw_white_pts = None  # CPU numpy
        self.yellow_tensor = None  # GPU tensor
        self.white_tensor = None   # GPU tensor
        
        self.sync_attempts = 0
        self.max_sync_attempts = 10 
        self.wp_path = os.path.join(os.path.dirname(__file__), "track_centerline_1x.npy")

    def ensure_synced(self):
        if self.yellow_tensor is not None and self.sync_attempts >= self.max_sync_attempts:
            return

        if self.sync_attempts == 0:
            print(f"[TrackManager] Collecting road markers for boundary detection...")
            self.collect_raw_marker_points()
            
            if self.raw_yellow_pts is not None:
                self.yellow_tensor = torch.tensor(self.raw_yellow_pts, device=self.device, dtype=torch.float32)
            if self.raw_white_pts is not None:
                self.white_tensor = torch.tensor(self.raw_white_pts, device=self.device, dtype=torch.float32)
            
            # Load real waypoints for lane-centering
            if os.path.exists(self.wp_path):
                print(f"[TrackManager] Loading waypoints from {self.wp_path}")
                wp_data = np.load(self.wp_path)
                self.waypoints = torch.tensor(wp_data, device=self.device, dtype=torch.float32)
            else:
                print(f"[TrackManager] WARNING: Waypoint file not found at {self.wp_path}. Using fallback.")
                self.waypoints = torch.tensor([[-16.25, 5.56, -1.57]], device=self.device)

        if self.sync_attempts < 500: # Keep trying to render for longer
            if self.sync_attempts % 100 == 0:
                print(f"[TrackManager] Visualizing {len(self.raw_yellow_pts) if self.raw_yellow_pts is not None else 0} yellow points...")
            self.refresh_visuals()
            self.sync_attempts += 1

    def collect_raw_marker_points(self):
        import omni.usd
        from pxr import Usd, UsdGeom, UsdShade, Gf
        stage = omni.usd.get_context().get_stage()
        if stage is None: return

        root_prim = stage.GetPrimAtPath("/World/envs/env_0")
        if not root_prim.IsValid(): root_prim = stage.GetPseudoRoot()

        xform_cache = UsdGeom.XformCache()
        try:
            env0_origin = xform_cache.GetLocalToWorldTransform(root_prim).ExtractTranslation()
        except:
            env0_origin = Gf.Vec3d(0,0,0)

        y_data, w_data = [], []
        for prim in Usd.PrimRange(root_prim):
            if not prim.IsA(UsdGeom.Mesh): continue
            path = str(prim.GetPath())
            binding_api = UsdShade.MaterialBindingAPI(prim)
            material, _ = binding_api.ComputeBoundMaterial()
            mat_path = str(material.GetPath()).lower() if material else ""
            
            search_str = (path + mat_path).lower()
            is_yellow = "yellow" in search_str
            is_white = "white" in search_str
            if not (is_yellow or is_white): continue
            
            # Use vertices to handle both small dots and large continuous meshes
            points_attr = UsdGeom.Mesh(prim).GetPointsAttr().Get()
            if not points_attr: continue
            
            xform = xform_cache.GetLocalToWorldTransform(prim)
            for p in points_attr:
                pw = xform.Transform(p)
                pl = [pw[0] - env0_origin[0], pw[1] - env0_origin[1], pw[2] - env0_origin[2]]
                
                # Store tuple of (path_string, coordinates) for sorting
                if is_yellow: y_data.append((path, pl))
                else: w_data.append((path, pl))

        if y_data: 
            y_data.sort(key=lambda x: x[0])
            y_pts_sorted = np.array([x[1] for x in y_data])
            # Use a slightly larger tolerance for uniqueness to clean up vertex soup
            _, unique_indices = np.unique(np.round(y_pts_sorted, 3), axis=0, return_index=True)
            y_pts_unique = y_pts_sorted[np.sort(unique_indices)]
            self.raw_yellow_pts = self._interpolate_path(y_pts_unique)
            print(f"[TrackManager] Found {len(y_data)} yellow prims, generated {len(self.raw_yellow_pts)} wall markers.")

        if w_data: 
            w_data.sort(key=lambda x: x[0])
            w_pts_sorted = np.array([x[1] for x in w_data])
            _, unique_indices = np.unique(np.round(w_pts_sorted, 3), axis=0, return_index=True)
            w_pts_unique = w_pts_sorted[np.sort(unique_indices)]
            self.raw_white_pts = self._interpolate_path(w_pts_unique)
            print(f"[TrackManager] Found {len(w_data)} white prims, generated {len(self.raw_white_pts)} wall markers.")

    def _interpolate_path(self, sorted_pts, resolution=0.1):
        """Fills gaps between sorted path points."""
        if len(sorted_pts) < 2: return sorted_pts
        dense_pts = []
        for i in range(len(sorted_pts) - 1):
            p1, p2 = sorted_pts[i], sorted_pts[i+1]
            dist = np.linalg.norm(p2 - p1)
            dense_pts.append(p1)
            if 0.0 < dist < 2.0: # Only interpolate reasonable gaps (skip jumps between non-sequential prims)
                num_steps = int(dist / resolution)
                for step in range(1, num_steps):
                    dense_pts.append(p1 + (p2 - p1) * (step / num_steps))
        dense_pts.append(sorted_pts[-1])
        return np.array(dense_pts)

    def compute_marker_distances(self, pos: torch.Tensor):
        """Returns distance to closest yellow and white markers (3D)."""
        self.ensure_synced()
        
        # Check all 3 axes (X, Y, Z) for high-fidelity boundary checks
        dist_y = torch.min(torch.cdist(pos[:, :3], self.yellow_tensor), dim=1)[0]
        dist_w = torch.min(torch.cdist(pos[:, :3], self.white_tensor), dim=1)[0]
        
        return dist_y, dist_w

    def refresh_visuals(self):
        # Use omni.debugDraw interface for guaranteed visibility in GUI
        try:
            import omni.debugdraw
            from pxr import Usd, UsdGeom
            draw = omni.debugdraw.get_debug_draw_interface()
            
            stage = omni.usd.get_context().get_stage()
            env0_prim = stage.GetPrimAtPath("/World/envs/env_0")
            if not env0_prim.IsValid(): return
            
            xform = UsdGeom.Xformable(env0_prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
            origin = xform.ExtractTranslation()

            if self.raw_yellow_pts is not None:
                pts = [(p[0]+origin[0], p[1]+origin[1], p[2]+origin[2]+0.1) for p in self.raw_yellow_pts]
                draw.draw_points(pts, [(1,1,0,1)]*len(pts), [10]*len(pts)) 
            if self.raw_white_pts is not None:
                pts = [(p[0]+origin[0], p[1]+origin[1], p[2]+origin[2]+0.1) for p in self.raw_white_pts]
                draw.draw_points(pts, [(1,1,1,1)]*len(pts), [10]*len(pts))
        except Exception as e:
            pass

    def compute_errors(self, pos: torch.Tensor, yaw: torch.Tensor, target_lane_offset: float = 0.22):
        """
        Returns distance to lane center and heading error using waypoints.
        target_lane_offset: Offset from the waypoint (double yellow line) to the lane center.
                           Negative is Right (for North-facing waypoints).
        """
        self.ensure_synced()

        # Find closest waypoints
        dists = torch.cdist(pos[:, :2], self.waypoints[:, :2])
        closest_idx = torch.argmin(dists, dim=1)
        closest_wp = self.waypoints[closest_idx]

        # 1. Lateral Error
        # Vector from waypoint to robot
        wp_to_robot = pos[:, :2] - closest_wp[:, :2]
        
        # Waypoint orientation
        wp_yaw = closest_wp[:, 2]
        
        # Normal vector to the path (rotate wp_yaw by +90 degrees)
        # For a right-handed system, if forward is (cos(yaw), sin(yaw)), 
        # left normal is (-sin(yaw), cos(yaw))
        normal_x = -torch.sin(wp_yaw)
        normal_y = torch.cos(wp_yaw)
        
        # Dot product with normal gives signed lateral error (positive = left of path)
        raw_lat_err = wp_to_robot[:, 0] * normal_x + wp_to_robot[:, 1] * normal_y
        
        # Apply lane offset (Shift target to center of right lane)
        lat_err = raw_lat_err - target_lane_offset

        # Debug lat_err for spawn verification
        if self.sync_attempts < 20 and pos.shape[0] > 0:
            print(f"[TrackManager] DEBUG Pos: {pos[0, :2].cpu().numpy()} | WP: {closest_wp[0, :2].cpu().numpy()} | LatErr: {lat_err[0].item():.4f}")

        # 2. Heading Error
        head_err = yaw - wp_yaw
        # Wrap to [-pi, pi]
        head_err = torch.atan2(torch.sin(head_err), torch.cos(head_err))
        
        # Handle bi-directional track (allow driving either way)
        import math
        head_err = torch.where(head_err > math.pi/2, head_err - math.pi, head_err)
        head_err = torch.where(head_err < -math.pi/2, head_err + math.pi, head_err)

        return lat_err, head_err

_TRACK_MANAGER = None
def get_track_manager(device: str = "cuda:0"):
    global _TRACK_MANAGER
    if _TRACK_MANAGER is None:
        _TRACK_MANAGER = TrackManager(device=device)
    return _TRACK_MANAGER
