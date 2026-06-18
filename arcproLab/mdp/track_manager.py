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

    def compute_marker_distances_with_positions(self, pos: torch.Tensor):
        """Same as compute_marker_distances but also returns the XY position of
        the closest yellow and white marker per env. Useful for debug logging
        at termination time.

        Returns:
            dist_y, dist_w        -- shape (N,)
            closest_y, closest_w  -- shape (N, 2) world XY (local frame, since
                                     `pos` is already env_origin-subtracted by
                                     the caller)
        """
        self.ensure_synced()

        y_dists = torch.cdist(pos[:, :2], self.yellow_tensor)  # (N, Ny)
        w_dists = torch.cdist(pos[:, :2], self.white_tensor)   # (N, Nw)

        dist_y, idx_y = torch.min(y_dists, dim=1)
        dist_w, idx_w = torch.min(w_dists, dim=1)

        closest_y = self.yellow_tensor[idx_y]  # (N, 2)
        closest_w = self.white_tensor[idx_w]   # (N, 2)

        return dist_y, dist_w, closest_y, closest_w

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
        """Privileged lateral & heading errors w.r.t. the lane.

        IMPORTANT — this is *active / privileged* signal extraction. It
        reads sim ground-truth pose (passed in by the caller) and the
        USD-scraped yellow/white marker point clouds. The values are
        consumed by the reward path only; the observation layer keeps
        slots 8 and 9 PVP-masked to 0.0 so the policy never sees this
        signal at deployment time. The policy must learn the geometry
        from camera + telemetry.

        Args:
            pos: (N, 3) robot position in env-0-local frame (already
                ``root_pos_w - env_origins`` by the caller).
            yaw: (N,) robot yaw angle, world-axes (radians).

        Returns:
            lat_err: (N,) signed lateral offset from the lane center.
                lane_center is the midpoint of the nearest yellow and
                nearest white marker; the sign is positive when the
                robot is offset toward the white marker (right curb)
                and negative toward yellow (centerline).
            head_err: (N,) signed angle between robot forward and the
                local lane tangent estimated from the two nearest
                yellow markers (disambiguated to point in roughly the
                same direction as the robot). Zero = aligned; +pi/2 =
                90 deg off; +/-pi = facing backwards.

        Edge cases (degenerate marker geometry, fewer than 2 yellow
        markers, etc.) fall through to zeros so reward functions stay
        well-defined; in those cases the lat_err / head_err signal is
        effectively absent for that step, not poisoned.
        """
        self.ensure_synced()

        N = pos.shape[0]
        zeros = torch.zeros(N, device=self.device)
        if (
            self.yellow_tensor is None
            or self.white_tensor is None
            or self.yellow_tensor.shape[0] < 2
            or self.white_tensor.shape[0] < 1
        ):
            return zeros, zeros

        pos_xy = pos[:, :2]

        # Nearest yellow and white markers per env.
        y_dists = torch.cdist(pos_xy, self.yellow_tensor)  # (N, Ny)
        w_dists = torch.cdist(pos_xy, self.white_tensor)   # (N, Nw)

        nearest_y_idx = torch.argmin(y_dists, dim=1)
        nearest_w_idx = torch.argmin(w_dists, dim=1)
        yp = self.yellow_tensor[nearest_y_idx]  # (N, 2)
        wp = self.white_tensor[nearest_w_idx]   # (N, 2)

        # --- lat_err: signed perpendicular distance from lane center. ---
        perp = wp - yp                                            # yellow -> white
        perp_norm = torch.linalg.norm(perp, dim=1, keepdim=True)
        perp_safe = perp / torch.clamp(perp_norm, min=1e-6)
        center = (yp + wp) * 0.5
        offset = pos_xy - center
        lat_err = (offset * perp_safe).sum(dim=1)                 # (N,)

        # --- head_err: signed angle between robot forward and lane tangent. ---
        # Lane tangent estimated by PCA on the K nearest yellow markers.
        # The "2 nearest" approach picks up any tightly-clustered yellow
        # feature (intersection crosswalks, arrows, lane-end stripes)
        # whose local axis is perpendicular to the actual lane — so head_err
        # comes back as 90 deg at spawn (verified). PCA over ~K nearest
        # yellow markers averages those out and recovers the lane direction.
        K = min(40, self.yellow_tensor.shape[0])
        sorted_y_idx = torch.topk(y_dists, k=K, dim=1, largest=False).indices
        # nearest_K: (N, K, 2)
        nearest_K = self.yellow_tensor[sorted_y_idx]
        mean_K = nearest_K.mean(dim=1, keepdim=True)
        centered = nearest_K - mean_K
        # cov: (N, 2, 2) symmetric PSD
        cov = torch.einsum("nki,nkj->nij", centered, centered) / K
        # eigh returns eigenvalues ascending; principal direction is last col.
        _eigvals, eigvecs = torch.linalg.eigh(cov)
        tangent_safe = eigvecs[:, :, -1]  # (N, 2)

        # Disambiguate tangent direction by aligning with robot forward.
        robot_fwd = torch.stack([torch.cos(yaw), torch.sin(yaw)], dim=1)  # (N, 2)
        same_dir = (tangent_safe * robot_fwd).sum(dim=1)
        flip = torch.where(same_dir < 0.0, -1.0, 1.0).unsqueeze(1)
        tangent_oriented = tangent_safe * flip

        # Signed angle: atan2(cross, dot) of (tangent, robot_fwd).
        cross = (
            tangent_oriented[:, 0] * robot_fwd[:, 1]
            - tangent_oriented[:, 1] * robot_fwd[:, 0]
        )
        dot = (tangent_oriented * robot_fwd).sum(dim=1)
        head_err = torch.atan2(cross, dot)                        # (N,)

        # Defensive NaN/Inf scrub so the reward path never sees bad floats.
        bad = ~(torch.isfinite(lat_err) & torch.isfinite(head_err))
        if bad.any():
            lat_err = torch.where(bad, torch.zeros_like(lat_err), lat_err)
            head_err = torch.where(bad, torch.zeros_like(head_err), head_err)

        return lat_err, head_err

_TRACK_MANAGER = None
def get_track_manager(device: str = "cuda:0"):
    global _TRACK_MANAGER
    if _TRACK_MANAGER is None:
        _TRACK_MANAGER = TrackManager(device=device)
    return _TRACK_MANAGER
