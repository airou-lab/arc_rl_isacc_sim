# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import torch
import numpy as np
import os

class TrackManager:
    """
    Robust Track Manager with VisualizationMarkers and Gap-Filling.
    Ensures 'Dense Walls' for reliable termination and visible debug spheres.
    """
    def __init__(self, device: str = "cuda:0"):
        self.device = device
        self.waypoints = None
        
        # Visualizers (Using reliable USD-based markers)
        self.visualizer_yellow = None
        self.visualizer_white = None
        self.visualizer_waypoints = None
        
        # Point groups
        self.raw_yellow_pts = None 
        self.raw_white_pts = None  
        self.yellow_tensor = None  
        self.white_tensor = None   
        
        self.sync_attempts = 0
        self.max_sync_attempts = 10 
        self.wp_path = os.path.join(os.path.dirname(__file__), "track_centerline_1x.npy")

    def ensure_synced(self):
        if self.yellow_tensor is not None and self.sync_attempts >= self.max_sync_attempts:
            return

        if self.sync_attempts == 0:
            print(f"[TrackManager] Building high-fidelity boundary markers...")
            self.collect_raw_marker_points()
            
            if self.raw_yellow_pts is not None:
                self.yellow_tensor = torch.tensor(self.raw_yellow_pts, device=self.device, dtype=torch.float32)
            if self.raw_white_pts is not None:
                self.white_tensor = torch.tensor(self.raw_white_pts, device=self.device, dtype=torch.float32)
            
            if os.path.exists(self.wp_path):
                self.waypoints = torch.tensor(np.load(self.wp_path), device=self.device, dtype=torch.float32)
            else:
                self.waypoints = torch.tensor([[-16.25, 5.56, -1.57]], device=self.device)

        # Persistent visuals for GUI
        if self.sync_attempts < 1000:
            self.refresh_visuals()
            self.sync_attempts += 1

    def collect_raw_marker_points(self):
        import omni.usd
        from pxr import Usd, UsdGeom, UsdShade, Gf
        stage = omni.usd.get_context().get_stage()
        root_prim = stage.GetPrimAtPath("/World/envs/env_0")
        if not root_prim.IsValid(): root_prim = stage.GetPseudoRoot()
        xform_cache = UsdGeom.XformCache()
        env0_origin = xform_cache.GetLocalToWorldTransform(root_prim).ExtractTranslation()

        y_data, w_data = [], []
        
        for prim in Usd.PrimRange(root_prim):
            if not prim.IsA(UsdGeom.Mesh): continue
            path = str(prim.GetPath())
            binding_api = UsdShade.MaterialBindingAPI(prim)
            mat, _ = binding_api.ComputeBoundMaterial()
            search_str = (path + str(mat.GetPath() if mat else "")).lower()
            
            is_y = "yellow" in search_str
            is_w = "white" in search_str
            if not (is_y or is_w): continue
            
            points_attr = UsdGeom.Mesh(prim).GetPointsAttr().Get()
            if not points_attr: continue
            
            xform = xform_cache.GetLocalToWorldTransform(prim)
            for p in points_attr:
                pw = xform.Transform(p)
                pl = [pw[0] - env0_origin[0], pw[1] - env0_origin[1], pw[2] - env0_origin[2]]
                # GROUND LOCK
                if -0.05 < pl[2] < 0.1:
                    if is_y: y_data.append((path, pl))
                    else: w_data.append((path, pl))

        def finalize_group(data_list, name):
            if not data_list: return None
            
            all_dense_pts = []
            resolution = 0.05
            
            for path, mesh_data in data_list:
                pts = np.array(mesh_data['points'])
                indices = mesh_data['indices']
                counts = mesh_data['counts']
                
                # Iterate through faces and their edges
                curr_idx = 0
                for count in counts:
                    face_indices = indices[curr_idx : curr_idx + count]
                    curr_idx += count
                    
                    # For each edge in the face (e.g., 0-1, 1-2, 2-0 for a triangle)
                    for i in range(count):
                        p1 = pts[face_indices[i]]
                        p2 = pts[face_indices[(i + 1) % count]]
                        
                        dist = np.linalg.norm(p2 - p1)
                        if dist > 0:
                            num_steps = max(1, int(dist / resolution))
                            for step in range(num_steps):
                                all_dense_pts.append(p1 + (p2 - p1) * (step / num_steps))
            
            if not all_dense_pts: return None
            final_pts = np.unique(np.round(np.array(all_dense_pts), 3), axis=0)
            print(f"[TrackManager] Finalized {name}: {len(final_pts)} points (Edge-Sampled).")
            return final_pts

        # Update collection to include mesh topology
        y_data, w_data = [], []
        for prim in Usd.PrimRange(root_prim):
            if not prim.IsA(UsdGeom.Mesh): continue
            path = str(prim.GetPath())
            binding_api = UsdShade.MaterialBindingAPI(prim)
            mat, _ = binding_api.ComputeBoundMaterial()
            search_str = (path + str(mat.GetPath() if mat else "")).lower()
            
            is_y = "yellow" in search_str
            is_w = "white" in search_str
            if not (is_y or is_w): continue
            
            mesh = UsdGeom.Mesh(prim)
            points_attr = mesh.GetPointsAttr().Get()
            indices_attr = mesh.GetFaceVertexIndicesAttr().Get()
            counts_attr = mesh.GetFaceVertexCountsAttr().Get()
            
            if not points_attr or not indices_attr: continue
            
            xform = xform_cache.GetLocalToWorldTransform(prim)
            # Transform points to Local Env Frame
            transformed_pts = []
            for p in points_attr:
                pw = xform.Transform(p)
                transformed_pts.append([pw[0] - env0_origin[0], pw[1] - env0_origin[1], pw[2] - env0_origin[2]])
            
            mesh_info = {'points': transformed_pts, 'indices': indices_attr, 'counts': counts_attr}
            if is_y: y_data.append((path, mesh_info))
            else: w_data.append((path, mesh_info))

        self.raw_yellow_pts = finalize_group(y_data, "Yellow")
        self.raw_white_pts = finalize_group(w_data, "White")

    def _interpolate_path(self, sorted_pts, resolution=0.05, max_gap=0.5):
        if len(sorted_pts) < 2: return sorted_pts
        dense_pts = []
        for i in range(len(sorted_pts) - 1):
            p1, p2 = sorted_pts[i], sorted_pts[i+1]
            dist = np.linalg.norm(p2 - p1)
            dense_pts.append(p1)
            # Only bridge small gaps (e.g. vertices in the same mesh)
            if 0.0 < dist < max_gap:
                num_steps = int(dist / resolution)
                for step in range(1, num_steps):
                    dense_pts.append(p1 + (p2 - p1) * (step / num_steps))
        dense_pts.append(sorted_pts[-1])
        return np.array(dense_pts)

    def compute_marker_distances(self, pos: torch.Tensor):
        self.ensure_synced()
        # High-fidelity 3D check
        dist_y = torch.min(torch.cdist(pos[:, :3], self.yellow_tensor), dim=1)[0]
        dist_w = torch.min(torch.cdist(pos[:, :3], self.white_tensor), dim=1)[0]
        return dist_y, dist_w

    def refresh_visuals(self):
        try:
            from isaaclab.markers import VisualizationMarkers, VisualizationMarkersCfg
            import isaaclab.sim as sim_utils
            from pxr import UsdGeom, Gf, Usd
            import omni.usd
            stage = omni.usd.get_context().get_stage()
            env0_prim = stage.GetPrimAtPath("/World/envs/env_0")
            origin = UsdGeom.Xformable(env0_prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation() if env0_prim.IsValid() else Gf.Vec3d(0,0,0)

            def create_v(path, color, radius=0.05):
                cfg = VisualizationMarkersCfg(prim_path=path, markers={"dot": sim_utils.SphereCfg(radius=radius, visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=color))})
                return VisualizationMarkers(cfg)

            def to_w(pts, z_off=0.1):
                if pts is None: return None
                w = torch.zeros((len(pts), 3), device=self.device)
                w[:, 0], w[:, 1], w[:, 2] = torch.tensor(pts[:, 0], device=self.device) + origin[0], torch.tensor(pts[:, 1], device=self.device) + origin[1], torch.tensor(pts[:, 2], device=self.device) + origin[2] + z_off
                return w

            if self.visualizer_yellow is None: self.visualizer_yellow = create_v("/World/Visuals/YellowSpheres", (1.0, 1.0, 0.0))
            if self.visualizer_white is None: self.visualizer_white = create_v("/World/Visuals/WhiteSpheres", (1.0, 1.0, 1.0))
            if self.visualizer_waypoints is None: self.visualizer_waypoints = create_v("/World/Visuals/CyanPath", (0.0, 1.0, 1.0), radius=0.08)

            if self.raw_yellow_pts is not None: self.visualizer_yellow.visualize(to_w(self.raw_yellow_pts))
            if self.raw_white_pts is not None: self.visualizer_white.visualize(to_w(self.raw_white_pts))
            if self.waypoints is not None:
                # IMPORTANT: waypoints are [X, Y, Yaw]. Force Z=0 for visualization.
                wp_np = self.waypoints.cpu().numpy()
                wp_viz = np.zeros((len(wp_np), 3))
                wp_viz[:, :2] = wp_np[:, :2]
                wp_viz[:, 2] = 0.0 # Force ground level
                self.visualizer_waypoints.visualize(to_w(wp_viz, z_off=0.15))
        except: pass

    def compute_errors(self, pos: torch.Tensor, yaw: torch.Tensor, target_lane_offset: float = 0.238):
        self.ensure_synced()
        dists = torch.cdist(pos[:, :2], self.waypoints[:, :2])
        closest_idx = torch.argmin(dists, dim=1)
        closest_wp = self.waypoints[closest_idx]
        wp_to_robot = pos[:, :2] - closest_wp[:, :2]
        wp_yaw = closest_wp[:, 2]
        normal_x, normal_y = -torch.sin(wp_yaw), torch.cos(wp_yaw)
        raw_lat_err = wp_to_robot[:, 0] * normal_x + wp_to_robot[:, 1] * normal_y
        lat_err, head_err = raw_lat_err - target_lane_offset, yaw - wp_yaw
        head_err = torch.atan2(torch.sin(head_err), torch.cos(head_err))
        import math
        head_err = torch.where(head_err > math.pi/2, head_err - math.pi, head_err)
        head_err = torch.where(head_err < -math.pi/2, head_err + math.pi, head_err)
        return lat_err, head_err

_TRACK_MANAGER = None
def get_track_manager(device: str = "cuda:0"):
    global _TRACK_MANAGER
    if _TRACK_MANAGER is None: _TRACK_MANAGER = TrackManager(device=device)
    return _TRACK_MANAGER
