# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import torch
import numpy as np
import os

class TrackManager:
    """
    Manages track waypoints by auto-centering between yellow and white road markers.
    Ensures LatErr = 0.0 is the exact center of the drivable lane.
    """
    def __init__(self, device: str = "cuda:0"):
        self.device = device
        self.waypoints = None
        self.visualizer = None
        self.left_visualizer = None
        self.right_visualizer = None
        
        self.raw_yellow_pts = None
        self.raw_white_pts = None
        self.sync_attempts = 0
        self.max_sync_attempts = 10 
        self.wp_path = os.path.join(os.path.dirname(__file__), "track_centerline_1x.npy")

    def ensure_synced(self):
        if self.waypoints is not None and self.sync_attempts >= self.max_sync_attempts:
            return

        if self.sync_attempts == 0:
            print(f"[TrackManager] Auto-centering waypoints between road markers...")
            self.collect_raw_marker_points()
            self.generate_centerline()
            
            if self.waypoints is None:
                print(f"[TrackManager] Procedural failed, loading from baseline file: {self.wp_path}")
                if os.path.exists(self.wp_path):
                    self.load_waypoints(self.wp_path)
                else:
                    self.waypoints = torch.tensor([[-16.25, 5.56, -1.57]], device=self.device)
        
        self.refresh_visuals()
        self.sync_attempts += 1

    def load_waypoints(self, path: str):
        data = np.load(path)
        data = data[::-1]
        data[:, 2] += np.pi
        data[:, 2] = np.arctan2(np.sin(data[:, 2]), np.cos(data[:, 2]))
        # Shift legacy waypoints to approximate center
        data[:, 0] -= 0.5625 
        self.waypoints = torch.tensor(data.copy(), device=self.device, dtype=torch.float32)

    def collect_raw_marker_points(self):
        import omni.usd
        from pxr import Usd, UsdGeom, UsdShade, Gf
        stage = omni.usd.get_context().get_stage()
        if stage is None: return

        env0_origin = Gf.Vec3d(0, 0, 0)
        env0_prim = stage.GetPrimAtPath("/World/envs/env_0")
        if env0_prim.IsValid():
            xform = UsdGeom.Xformable(env0_prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
            env0_origin = xform.ExtractTranslation()

        y_pts, w_pts = [], []
        for prim in stage.Traverse():
            if not prim.IsA(UsdGeom.Mesh): continue
            path = str(prim.GetPath()).lower()
            
            # Check material for color naming
            mat_path = ""
            binding_api = UsdShade.MaterialBindingAPI(prim)
            material, _ = binding_api.ComputeBoundMaterial()
            if material: mat_path = str(material.GetPath()).lower()
            
            search_str = path + mat_path
            is_yellow = "yellow" in search_str
            is_white = "white" in search_str
            
            if not (is_yellow or is_white): continue
            
            points_attr = UsdGeom.Mesh(prim).GetPointsAttr().Get()
            if not points_attr: continue
            
            xform = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
            for p in points_attr:
                pw = xform.Transform(p)
                pl = [pw[0] - env0_origin[0], pw[1] - env0_origin[1]]
                if is_yellow: y_pts.append(pl)
                else: w_pts.append(pl)

        if y_pts: 
            self.raw_yellow_pts = np.unique(np.round(np.array(y_pts), 2), axis=0)
            print(f"[TrackManager] Found {len(self.raw_yellow_pts)} yellow markers.")
        if w_pts: 
            self.raw_white_pts = np.unique(np.round(np.array(w_pts), 2), axis=0)
            print(f"[TrackManager] Found {len(self.raw_white_pts)} white markers.")

    def generate_centerline(self):
        if self.raw_yellow_pts is None or self.raw_white_pts is None:
            return

        y_pts = self.raw_yellow_pts
        w_pts = self.raw_white_pts
        center_pts = []
        for yp in y_pts:
            dists = np.sum((w_pts - yp)**2, axis=1)
            closest_w = w_pts[np.argmin(dists)]
            center_pts.append((yp + closest_w) / 2.0)
            
        center_pts = np.array(center_pts)
        spawn_target = np.array([-16.25, 5.56])
        ordered = []
        dists = np.sum((center_pts - spawn_target)**2, axis=1)
        curr_idx = np.argmin(dists)
        visited = set([curr_idx])
        ordered.append(center_pts[curr_idx])
        
        while len(visited) < len(center_pts):
            last = ordered[-1]
            d = np.sum((center_pts - last)**2, axis=1)
            for v in visited: d[v] = np.inf
            next_idx = np.argmin(d)
            if d[next_idx] > 2.0**2: break 
            ordered.append(center_pts[next_idx])
            visited.add(next_idx)
            
        ordered = np.array(ordered)
        final_wps = []
        for i in range(len(ordered)):
            pt = ordered[i]
            next_pt = ordered[i+1] if i < len(ordered)-1 else pt + (pt - ordered[i-1])
            dx, dy = next_pt[0]-pt[0], next_pt[1]-pt[1]
            yaw = np.arctan2(dy, dx)
            final_wps.append([pt[0], pt[1], yaw])
            
        self.waypoints = torch.tensor(final_wps, device=self.device, dtype=torch.float32)
        print(f"[TrackManager] Generated {len(self.waypoints)} centered waypoints.")

    def refresh_visuals(self):
        if self.waypoints is None: return
        try:
            from isaaclab.markers import VisualizationMarkers, VisualizationMarkersCfg
            import isaaclab.sim as sim_utils
            import omni.usd
            from pxr import UsdGeom, Gf

            stage = omni.usd.get_context().get_stage()
            env0_origin = Gf.Vec3d(0, 0, 0)
            env0_prim = stage.GetPrimAtPath("/World/envs/env_0")
            if env0_prim.IsValid():
                xform = UsdGeom.Xformable(env0_prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
                env0_origin = xform.ExtractTranslation()

            def get_cfg(path, color, scale=0.1):
                return VisualizationMarkersCfg(prim_path=path, markers={"dot": sim_utils.SphereCfg(radius=scale, 
                    visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=color))})

            if self.left_visualizer is None:
                self.left_visualizer = VisualizationMarkers(get_cfg("/Visuals/RawYellowMarkers", (1.0, 1.0, 0.0), 0.15))
            if self.right_visualizer is None:
                self.right_visualizer = VisualizationMarkers(get_cfg("/Visuals/RawWhiteMarkers", (1.0, 1.0, 1.0), 0.15))
            if self.visualizer is None:
                self.visualizer = VisualizationMarkers(get_cfg("/Visuals/TrackWaypoints", (0.0, 1.0, 1.0), 0.1))

            def to_world(lpts, height=0.1):
                w = torch.zeros((len(lpts), 3), device=self.device)
                w[:, 0] = torch.tensor(lpts[:, 0], device=self.device) + env0_origin[0]
                w[:, 1] = torch.tensor(lpts[:, 1], device=self.device) + env0_origin[1]
                w[:, 2] = env0_origin[2] + height
                return w

            if self.raw_yellow_pts is not None: self.left_visualizer.visualize(to_world(self.raw_yellow_pts))
            if self.raw_white_pts is not None: self.right_visualizer.visualize(to_world(self.raw_white_pts))
            self.visualizer.visualize(to_world(self.waypoints[::5, :2].cpu().numpy(), height=0.2))
        except:
            pass

    def compute_errors(self, pos: torch.Tensor, yaw: torch.Tensor):
        self.ensure_synced()
        dists = torch.cdist(pos[:, :2], self.waypoints[:, :2])
        closest_indices = torch.argmin(dists, dim=1)
        wp_data = self.waypoints[closest_indices]
        
        wp_yaw = wp_data[:, 2]
        head_err = torch.atan2(torch.sin(yaw - wp_yaw), torch.cos(yaw - wp_yaw))
        v_wp_to_rob = pos[:, :2] - wp_data[:, :2]
        v_normal = torch.stack([-torch.sin(wp_yaw), torch.cos(wp_yaw)], dim=-1)
        lat_err = torch.sum(v_wp_to_rob * v_normal, dim=-1)
        return lat_err, head_err

_TRACK_MANAGER = None
def get_track_manager(device: str = "cuda:0"):
    global _TRACK_MANAGER
    if _TRACK_MANAGER is None:
        _TRACK_MANAGER = TrackManager(device=device)
    return _TRACK_MANAGER
