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
            
            # Keep a dummy waypoint for reward functions that expect it (until we refactor rewards)
            self.waypoints = torch.tensor([[-16.25, 5.56, -1.57]], device=self.device)

        if self.sync_attempts < 100:
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

        y_pts, w_pts = [], []
        for prim in Usd.PrimRange(root_prim):
            if not prim.IsA(UsdGeom.Mesh): continue
            path = str(prim.GetPath()).lower()
            binding_api = UsdShade.MaterialBindingAPI(prim)
            material, _ = binding_api.ComputeBoundMaterial()
            mat_path = str(material.GetPath()).lower() if material else ""
            
            search_str = path + mat_path
            is_yellow = "yellow" in search_str
            is_white = "white" in search_str
            if not (is_yellow or is_white): continue
            
            points_attr = UsdGeom.Mesh(prim).GetPointsAttr().Get()
            if not points_attr: continue
            
            xform = xform_cache.GetLocalToWorldTransform(prim)
            for p in points_attr:
                pw = xform.Transform(p)
                pl = [pw[0] - env0_origin[0], pw[1] - env0_origin[1]]
                if is_yellow: y_pts.append(pl)
                else: w_pts.append(pl)

        if y_pts: self.raw_yellow_pts = np.unique(np.round(np.array(y_pts), 2), axis=0)
        if w_pts: self.raw_white_pts = np.unique(np.round(np.array(w_pts), 2), axis=0)

    def compute_marker_distances(self, pos: torch.Tensor):
        """Returns distance to closest yellow and white markers."""
        self.ensure_synced()
        
        dist_y = torch.min(torch.cdist(pos[:, :2], self.yellow_tensor), dim=1)[0]
        dist_w = torch.min(torch.cdist(pos[:, :2], self.white_tensor), dim=1)[0]
        
        return dist_y, dist_w

    def refresh_visuals(self):
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

            def get_cfg(path, color):
                return VisualizationMarkersCfg(prim_path=path, markers={"dot": sim_utils.SphereCfg(radius=0.03, 
                    visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=color))})

            if self.left_visualizer is None:
                self.left_visualizer = VisualizationMarkers(get_cfg("/Visuals/YellowBoundaries", (1.0, 1.0, 0.0)))
            if self.right_visualizer is None:
                self.right_visualizer = VisualizationMarkers(get_cfg("/Visuals/WhiteBoundaries", (1.0, 1.0, 1.0)))

            def to_world(lpts):
                w = torch.zeros((len(lpts), 3), device=self.device)
                w[:, 0] = torch.tensor(lpts[:, 0], device=self.device, dtype=torch.float32) + env0_origin[0]
                w[:, 1] = torch.tensor(lpts[:, 1], device=self.device, dtype=torch.float32) + env0_origin[1]
                w[:, 2] = env0_origin[2] + 0.2 # High visibility
                return w

            if self.raw_yellow_pts is not None: self.left_visualizer.visualize(to_world(self.raw_yellow_pts))
            if self.raw_white_pts is not None: self.right_visualizer.visualize(to_world(self.raw_white_pts))
        except:
            pass

    def compute_errors(self, pos: torch.Tensor, yaw: torch.Tensor):
        """Returns distance to lane center and heading error."""
        self.ensure_synced()

        # Lane Center Approximation: (DistYellow - DistWhite) / 2
        dist_y, dist_w = self.compute_marker_distances(pos)
        lat_err = (dist_y - dist_w) * 0.5

        # Calculate target_yaw dynamically from track markers
        # We use the two closest white markers to find the track direction
        dists, indices = torch.topk(torch.cdist(pos[:, :2], self.white_tensor), k=2, largest=False)
        p1 = self.white_tensor[indices[:, 0]]
        p2 = self.white_tensor[indices[:, 1]]

        # Track direction vector (p1 to p2, or p2 to p1)
        # We pick the direction that is generally aligned with current yaw
        v1 = p2 - p1
        v2 = p1 - p2

        target_yaw1 = torch.atan2(v1[:, 1], v1[:, 0])
        target_yaw2 = torch.atan2(v2[:, 1], v2[:, 0])

        # Pick the one closest to current yaw to handle bi-directional track ambiguity
        diff1 = torch.abs(torch.atan2(torch.sin(yaw - target_yaw1), torch.cos(yaw - target_yaw1)))
        diff2 = torch.abs(torch.atan2(torch.sin(yaw - target_yaw2), torch.cos(yaw - target_yaw2)))

        target_yaw = torch.where(diff1 < diff2, target_yaw1, target_yaw2)
        head_err = yaw - target_yaw

        # Wrap heading error to [-pi, pi]
        head_err = torch.atan2(torch.sin(head_err), torch.cos(head_err))

        return lat_err, head_err

_TRACK_MANAGER = None
def get_track_manager(device: str = "cuda:0"):
    global _TRACK_MANAGER
    if _TRACK_MANAGER is None:
        _TRACK_MANAGER = TrackManager(device=device)
    return _TRACK_MANAGER
