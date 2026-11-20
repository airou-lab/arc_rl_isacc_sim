# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Robust Track Manager with VisualizationMarkers and Gap-Filling.

Ensures 'Dense Walls' for reliable termination and visible debug spheres.
Modularized per Grug principles: USD mesh extraction in track_usd_extractor.py,
visualization in track_visualizer.py, core runtime navigation in this module.
"""

import os
import sys

import numpy as np
import torch

try:
    from .track_usd_extractor import (
        collect_raw_marker_points_from_stage,
        sample_waypoints as _sample_waypoints,
    )
    from .track_visualizer import refresh_track_visuals
except ImportError:
    try:
        from mdp.track_usd_extractor import (
            collect_raw_marker_points_from_stage,
            sample_waypoints as _sample_waypoints,
        )
        from mdp.track_visualizer import refresh_track_visuals
    except ImportError:
        from arcproLab.mdp.track_usd_extractor import (
            collect_raw_marker_points_from_stage,
            sample_waypoints as _sample_waypoints,
        )
        from arcproLab.mdp.track_visualizer import refresh_track_visuals


class TrackManager:
    """Robust Track Manager with VisualizationMarkers and Gap-Filling.

    Ensures 'Dense Walls' for reliable termination and visible debug spheres.
    """

    def __init__(self, device: str | torch.device = "cuda:0"):
        if hasattr(device, "device"):
            device = device.device
        self.device = device
        self.waypoints = None
        self.last_indices = None  # Tensor (num_envs,) to track closest waypoints

        # Performance: Only enable visuals if explicitly requested via --debug flag
        self.debug = "--debug" in sys.argv

        # Visualizers (Using reliable USD-based markers)
        self.visualizer_yellow = None
        self.visualizer_white = None
        self.visualizer_gate = None
        self.visualizer_waypoints = None
        self.visualizer_target = None

        # Point groups
        self.raw_yellow_pts = None
        self.raw_white_pts = None
        self.raw_gate_pts = None
        self.yellow_tensor = None
        self.white_tensor = None
        self.gate_tensor = None
        self.curvature_tensor = None

        # Contiguous 2D XY views and per-pos cdist cache for fast exact queries
        self._yellow_xy = None
        self._white_xy = None
        self._gate_xy = None
        self._waypoints_xy = None
        self._cached_marker_pos_xy = None
        self._cached_dist_y = None
        self._cached_dist_w = None
        self._cached_dist_g = None

        # Multi-branch tensor bank (Gate 1: branch 0=Left/-1.0, 1=Straight/0.0, 2=Right/+1.0)
        self.branch_waypoints = None
        self.branch_waypoints_xy = None
        self.branch_curvatures = None
        self.branch_seg_bounds = None
        self.last_branch_ids = None
        self.branch_switched_mask = None

        self.sync_attempts = 0
        self.max_sync_attempts = 10
        self.wp_path = os.path.join(os.path.dirname(__file__), "track_centerline.npy")
        self.cache_path = os.path.join(
            os.path.dirname(__file__), "track_boundaries_1x.npz"
        )

    def ensure_synced(self, target_lane_offset: float = 0.0):
        if (
            self.yellow_tensor is not None
            and self.sync_attempts >= self.max_sync_attempts
        ):
            return

        if self.sync_attempts == 0:
            # TRY LOADING CACHE FIRST
            if self.load_cache():
                print(f"[TrackManager] Loaded boundaries from cache: {self.cache_path}")
            else:
                print(
                    "[TrackManager] Building high-fidelity boundary markers from USD (slow)..."
                )
                self.collect_raw_marker_points()
                self.save_cache()

            # Convert to Tensors and pre-materialize contiguous 2D XY views
            if self.raw_yellow_pts is not None:
                self.yellow_tensor = torch.tensor(
                    self.raw_yellow_pts, device=self.device, dtype=torch.float32
                )
                self._yellow_xy = self.yellow_tensor[:, :2].contiguous()
            if self.raw_white_pts is not None:
                self.white_tensor = torch.tensor(
                    self.raw_white_pts, device=self.device, dtype=torch.float32
                )
                self._white_xy = self.white_tensor[:, :2].contiguous()
            if self.raw_gate_pts is not None:
                self.gate_tensor = torch.tensor(
                    self.raw_gate_pts, device=self.device, dtype=torch.float32
                )
                self._gate_xy = self.gate_tensor[:, :2].contiguous()

            if os.path.exists(self.wp_path):
                wp_np = np.load(self.wp_path)
                self.waypoints = torch.tensor(
                    wp_np, device=self.device, dtype=torch.float32
                )
                self._waypoints_xy = self.waypoints[:, :2].contiguous()

                # Pre-calculate Curvature (Kappa = dYaw / ds)
                n_wps = len(wp_np)
                kappa = np.zeros(n_wps)
                for i in range(n_wps):
                    i1 = (i + 1) % n_wps
                    dyaw = wp_np[i1, 2] - wp_np[i, 2]
                    # Handle yaw wrap-around
                    while dyaw > np.pi:
                        dyaw -= 2 * np.pi
                    while dyaw < -np.pi:
                        dyaw += 2 * np.pi

                    ds = np.linalg.norm(wp_np[i1, :2] - wp_np[i, :2])
                    if ds > 0:
                        kappa[i] = dyaw / ds

                self.curvature_tensor = torch.tensor(
                    kappa, device=self.device, dtype=torch.float32
                )
                print(f"[TrackManager] Calculated curvature for {n_wps} waypoints.")
            else:
                self.waypoints = torch.tensor(
                    [[-16.25, 5.56, -1.57]], device=self.device
                )
                self._waypoints_xy = self.waypoints[:, :2].contiguous()
                self.curvature_tensor = torch.zeros(1, device=self.device)

            self._build_multi_branch_bank()

        self.sync_attempts += 1
        # Persistent visuals for GUI (Only if --debug is set)
        if self.debug and self.sync_attempts <= 1:
            self.refresh_visuals(target_lane_offset=target_lane_offset)

    def load_cache(self):
        if not os.path.exists(self.cache_path):
            return False
        try:
            data = np.load(self.cache_path)
            self.raw_yellow_pts = data["yellow"]
            self.raw_white_pts = data["white"]
            self.raw_gate_pts = data["gate"]
            return True
        except Exception as e:
            print(f"[TrackManager] Cache load failed: {e}")
            return False

    def save_cache(self):
        try:
            y = self.raw_yellow_pts if self.raw_yellow_pts is not None else np.array([])
            w = self.raw_white_pts if self.raw_white_pts is not None else np.array([])
            g = self.raw_gate_pts if self.raw_gate_pts is not None else np.array([])
            np.savez(self.cache_path, yellow=y, white=w, gate=g)
            print(f"[TrackManager] Saved boundaries to cache: {self.cache_path}")
        except Exception as e:
            print(f"[TrackManager] Cache save failed: {e}")

    def collect_raw_marker_points(self):
        """Extracts boundary points from the USD stage using track_usd_extractor."""
        (
            self.raw_yellow_pts,
            self.raw_white_pts,
            self.raw_gate_pts,
        ) = collect_raw_marker_points_from_stage()

    def sample_waypoints_from_usd(self, density=0.1):
        """Generates track centerline waypoints from USD boundary markers using a KD-Tree."""
        if self.raw_yellow_pts is None or self.raw_white_pts is None:
            self.collect_raw_marker_points()

        final_wps = _sample_waypoints(
            self.raw_yellow_pts[:, :2], self.raw_white_pts[:, :2]
        )
        n = len(final_wps)

        self.waypoints = torch.tensor(
            final_wps, device=self.device, dtype=torch.float32
        )
        self._waypoints_xy = self.waypoints[:, :2].contiguous()
        print(f"[TrackManager] Generated {n} sequenced waypoints.")

    def save_waypoints(self, path):
        if self.waypoints is not None:
            np.save(path, self.waypoints.cpu().numpy())
            print(f"[TrackManager] Saved waypoints to {path}")

    def _build_multi_branch_bank(self) -> None:
        """Pre-materialize the (3, N, 3) branch waypoint bank and (3, N) curvature bank.

        Branch IDs (IsaacLab MDP sign convention):
          0: LEFT     (turn_token = -1.0) -> canonical closed-loop circuit (self.waypoints)
          1: STRAIGHT (turn_token =  0.0) -> straight-through intersection corridors (junction_55, C2, junction_18)
          2: RIGHT    (turn_token = +1.0) -> 90-deg right-turn intersection corridors (R=0.50m, kappa=-2.0 m^-1)
        """
        if self.waypoints is None or len(self.waypoints) == 0:
            return

        n_wps = int(self.waypoints.shape[0])
        ds = 0.0102826

        # Branch 0: Canonical Left-turn loop
        b0_wps = self.waypoints.detach().cpu().numpy().astype(np.float32)
        b0_kappa = (
            self.curvature_tensor.detach().cpu().numpy().astype(np.float32)
            if self.curvature_tensor is not None
            else np.zeros(n_wps, dtype=np.float32)
        )
        b0_bounds = np.zeros((n_wps, 2), dtype=np.int64)
        b0_bounds[:, 0] = 0
        b0_bounds[:, 1] = max(0, n_wps - 1)

        if n_wps < 100:
            # Minimal fallback for single-point dummy waypoints
            self.branch_waypoints = (
                self.waypoints.unsqueeze(0).expand(3, -1, -1).contiguous()
            )
            self.branch_waypoints_xy = self.branch_waypoints[:, :, :2].contiguous()
            self.branch_curvatures = (
                self.curvature_tensor.unsqueeze(0).expand(3, -1).contiguous()
            )
            b_bounds_t = torch.tensor(
                b0_bounds, device=self.device, dtype=torch.long
            )
            self.branch_seg_bounds = (
                b_bounds_t.unsqueeze(0).expand(3, -1, -1).contiguous()
            )
            return

        # Partition n_wps (5000) into 3 disjoint intersection corridors with 0 boundary violations (< 0.170m):
        #   Segment A (0 .. n_a-1):         West Runway Southbound through junction_55 (-15.971, -6.875)
        #   Segment B (n_a .. n_a+n_b-1):   South Straightaway Eastbound through C2 (-5.500, -7.100)
        #   Segment C (n_a+n_b .. n_wps-1): Top North Straightaway Westbound through junction_18 (-15.969, +6.875)
        n_a = min(1820, n_wps // 3 + 154)
        n_b = min(1700, (n_wps - n_a) // 2 + 110)
        n_c = n_wps - n_a - n_b

        b1_wps = np.zeros((n_wps, 3), dtype=np.float32)
        b1_kappa = np.zeros(n_wps, dtype=np.float32)
        b1_bounds = np.zeros((n_wps, 2), dtype=np.int64)

        # Branch 1 (STRAIGHT, kappa = 0.0):
        # Seg S-A: Southbound along X = -16.205, Y = +5.50 -> -13.21 (tangent = -pi/2 -> wp_yaw = +pi/2)
        idx_a = np.arange(n_a, dtype=np.float32)
        b1_wps[:n_a, 0] = -16.205
        b1_wps[:n_a, 1] = 5.500 - idx_a * ds
        b1_wps[:n_a, 2] = float(np.pi / 2.0)
        b1_bounds[:n_a, 0] = 0
        b1_bounds[:n_a, 1] = n_a - 1

        # Seg S-B: Eastbound along Y = -7.100, X = -15.30 -> +2.18 (tangent = 0.0 -> wp_yaw = pi)
        idx_b = np.arange(n_b, dtype=np.float32)
        s_b = n_a
        e_b = n_a + n_b
        b1_wps[s_b:e_b, 0] = -15.300 + idx_b * ds
        b1_wps[s_b:e_b, 1] = -7.100
        b1_wps[s_b:e_b, 2] = float(np.pi)
        b1_bounds[s_b:e_b, 0] = s_b
        b1_bounds[s_b:e_b, 1] = e_b - 1

        # Seg S-C: Westbound along Y = +7.100, X = -7.20 -> -22.42 (tangent = pi -> wp_yaw = 0.0)
        idx_c = np.arange(n_c, dtype=np.float32)
        s_c = e_b
        b1_wps[s_c:, 0] = -7.200 - idx_c * ds
        b1_wps[s_c:, 1] = 7.100
        b1_wps[s_c:, 2] = 0.0
        b1_bounds[s_c:, 0] = s_c
        b1_bounds[s_c:, 1] = n_wps - 1

        # Branch 2 (RIGHT, 90-deg right turns with R = 0.50m, kappa = -2.0 m^-1):
        b2_wps = np.zeros((n_wps, 3), dtype=np.float32)
        b2_kappa = np.zeros(n_wps, dtype=np.float32)
        b2_bounds = b1_bounds.copy()
        r_turn = 0.50
        arc_len = float(0.5 * np.pi * r_turn)
        n_arc = max(10, int(round(arc_len / ds)))

        def _populate_right_corridor(
            start_idx: int,
            count: int,
            p_start: tuple[float, float],
            dir_in: tuple[float, float],
            yaw_in: float,
            n_approach: int,
            arc_center: tuple[float, float],
            theta_start: float,
            theta_end: float,
            dir_out: tuple[float, float],
            yaw_out: float,
        ) -> None:
            n_app = min(n_approach, count - n_arc - 1)
            n_ex = max(0, count - n_app - n_arc)
            # 1. Approach straight
            for k in range(n_app):
                gi = start_idx + k
                b2_wps[gi, 0] = p_start[0] + k * ds * dir_in[0]
                b2_wps[gi, 1] = p_start[1] + k * ds * dir_in[1]
                b2_wps[gi, 2] = yaw_in
                b2_kappa[gi] = 0.0
            # 2. 90-deg Right circular arc (kappa = -1 / R = -2.0 m^-1)
            thetas = np.linspace(theta_start, theta_end, n_arc, endpoint=False)
            for k, th in enumerate(thetas):
                gi = start_idx + n_app + k
                b2_wps[gi, 0] = arc_center[0] + r_turn * float(np.cos(th))
                b2_wps[gi, 1] = arc_center[1] + r_turn * float(np.sin(th))
                # Clockwise tangent is (sin(th), -cos(th)); wp_yaw = tangent_yaw + pi = atan2(cos(th), -sin(th))
                tx_tan = float(np.sin(th))
                ty_tan = float(-np.cos(th))
                wp_y = float(np.arctan2(-ty_tan, -tx_tan))
                b2_wps[gi, 2] = wp_y
                b2_kappa[gi] = -1.0 / r_turn
            # 3. Exit straight
            p_exit_0 = (
                arc_center[0] + r_turn * float(np.cos(theta_end)),
                arc_center[1] + r_turn * float(np.sin(theta_end)),
            )
            for k in range(n_ex):
                gi = start_idx + n_app + n_arc + k
                b2_wps[gi, 0] = p_exit_0[0] + k * ds * dir_out[0]
                b2_wps[gi, 1] = p_exit_0[1] + k * ds * dir_out[1]
                b2_wps[gi, 2] = yaw_out
                b2_kappa[gi] = 0.0

        # Seg R-A: junction_55 Right turn (Southbound X=-16.205, Y=+5.50..-6.156 -> Westbound Y=-6.656)
        n_app_a = int(round((5.500 - (-6.156)) / ds))
        _populate_right_corridor(
            start_idx=0,
            count=n_a,
            p_start=(-16.205, 5.500),
            dir_in=(0.0, -1.0),
            yaw_in=float(np.pi / 2.0),
            n_approach=n_app_a,
            arc_center=(-16.705, -6.156),
            theta_start=0.0,
            theta_end=float(-0.5 * np.pi),
            dir_out=(-1.0, 0.0),
            yaw_out=0.0,
        )

        # Seg R-B: Sector 4 / C2 Right turn (Eastbound Y=-7.100, X=-15.30..-5.50 -> Southbound X=-5.00)
        n_app_b = int(round((-5.500 - (-15.300)) / ds))
        _populate_right_corridor(
            start_idx=s_b,
            count=n_b,
            p_start=(-15.300, -7.100),
            dir_in=(1.0, 0.0),
            yaw_in=float(np.pi),
            n_approach=n_app_b,
            arc_center=(-5.500, -7.600),
            theta_start=float(0.5 * np.pi),
            theta_end=0.0,
            dir_out=(0.0, -1.0),
            yaw_out=float(np.pi / 2.0),
        )

        # Seg R-C: junction_18 Right turn (Westbound Y=+7.100, X=-7.20..-15.252 -> Northbound X=-15.752, Y<=13.98)
        n_app_c = int(round((-7.200 - (-15.252)) / ds))
        _populate_right_corridor(
            start_idx=s_c,
            count=n_c,
            p_start=(-7.200, 7.100),
            dir_in=(-1.0, 0.0),
            yaw_in=0.0,
            n_approach=n_app_c,
            arc_center=(-15.252, 7.600),
            theta_start=float(-0.5 * np.pi),
            theta_end=float(-np.pi),
            dir_out=(0.0, 1.0),
            yaw_out=float(-np.pi / 2.0),
        )

        stacked_wps = np.stack([b0_wps, b1_wps, b2_wps], axis=0)
        stacked_kappa = np.stack([b0_kappa, b1_kappa, b2_kappa], axis=0)
        stacked_bounds = np.stack([b0_bounds, b1_bounds, b2_bounds], axis=0)

        self.branch_waypoints = torch.tensor(
            stacked_wps, device=self.device, dtype=torch.float32
        ).contiguous()
        self.branch_waypoints_xy = self.branch_waypoints[:, :, :2].contiguous()
        self.branch_curvatures = torch.tensor(
            stacked_kappa, device=self.device, dtype=torch.float32
        ).contiguous()
        self.branch_seg_bounds = torch.tensor(
            stacked_bounds, device=self.device, dtype=torch.long
        ).contiguous()

    def resolve_branch_ids(
        self,
        num_envs: int,
        turn_token: torch.Tensor | float | int | None = None,
        branch_ids: torch.Tensor | None = None,
    ) -> torch.Tensor:
        """Map turn_token {-1.0, 0.0, +1.0} or explicit branch_ids {0, 1, 2} to (num_envs,) long tensor."""
        if branch_ids is not None:
            b = branch_ids.to(device=self.device, dtype=torch.long).view(-1)
            if b.shape[0] == 1 and num_envs > 1:
                b = b.expand(num_envs)
            elif b.shape[0] != num_envs:
                b = (
                    b[:num_envs]
                    if b.shape[0] > num_envs
                    else torch.nn.functional.pad(
                        b, (0, num_envs - b.shape[0]), value=0
                    )
                )
            return torch.clamp(b, min=0, max=2)
        if turn_token is None:
            return torch.zeros(num_envs, device=self.device, dtype=torch.long)
        if not isinstance(turn_token, torch.Tensor):
            tt = torch.full(
                (num_envs,), float(turn_token), device=self.device, dtype=torch.float32
            )
        else:
            tt = turn_token.to(device=self.device, dtype=torch.float32).view(-1)
            if tt.shape[0] == 1 and num_envs > 1:
                tt = tt.expand(num_envs)
            elif tt.shape[0] != num_envs:
                tt = (
                    tt[:num_envs]
                    if tt.shape[0] > num_envs
                    else torch.nn.functional.pad(
                        tt, (0, num_envs - tt.shape[0]), value=-1.0
                    )
                )
        return torch.where(
            tt < -0.5,
            torch.zeros(num_envs, device=self.device, dtype=torch.long),
            torch.where(
                tt > 0.5,
                torch.full((num_envs,), 2, device=self.device, dtype=torch.long),
                torch.ones(num_envs, device=self.device, dtype=torch.long),
            ),
        )

    def reset_indices(
        self,
        env_ids: torch.Tensor | None = None,
        new_pos: torch.Tensor | None = None,
        branch_ids: torch.Tensor | None = None,
    ) -> None:
        """Reset cached marker distances and optionally re-index waypoints on reset."""
        self._cached_marker_pos_xy = None
        self.ensure_synced()
        if new_pos is not None and self.waypoints is not None and len(self.waypoints) > 0:
            pos_xy = new_pos[:, :2].to(device=self.device, dtype=torch.float32)
            n_resets = pos_xy.shape[0]
            b_ids = self.resolve_branch_ids(n_resets, branch_ids=branch_ids)
            if bool((b_ids == 0).all().item()):
                wp_xy = (
                    self._waypoints_xy
                    if self._waypoints_xy is not None
                    else self.waypoints[:, :2]
                )
                dists = torch.cdist(pos_xy, wp_xy)
                closest_idx = torch.argmin(dists, dim=1)
            else:
                if self.branch_waypoints_xy is None:
                    self._build_multi_branch_bank()
                active_wp_xy = self.branch_waypoints_xy[b_ids]
                dists = torch.norm(pos_xy[:, None, :] - active_wp_xy, dim=2)
                closest_idx = torch.argmin(dists, dim=1)

            if env_ids is None or self.last_indices is None:
                self.last_indices = closest_idx
                self.last_branch_ids = b_ids.clone()
                self.branch_switched_mask = torch.zeros_like(
                    closest_idx, dtype=torch.bool
                )
            else:
                self.last_indices[env_ids] = closest_idx
                if (
                    self.last_branch_ids is None
                    or self.last_branch_ids.shape != self.last_indices.shape
                ):
                    self.last_branch_ids = torch.zeros_like(self.last_indices)
                self.last_branch_ids[env_ids] = b_ids
                if (
                    self.branch_switched_mask is None
                    or self.branch_switched_mask.shape != self.last_indices.shape
                ):
                    self.branch_switched_mask = torch.zeros_like(
                        self.last_indices, dtype=torch.bool
                    )
                self.branch_switched_mask[env_ids] = False
        elif env_ids is None:
            self.last_indices = None
            self.last_branch_ids = None
            self.branch_switched_mask = None

    def get_nearest_waypoint(
        self,
        pos: torch.Tensor,
        turn_token: torch.Tensor | float | int | None = None,
        branch_ids: torch.Tensor | None = None,
    ) -> torch.Tensor:
        """Return nearest waypoint indices for the given (N, 2+) positions."""
        self.ensure_synced()
        pos_xy = pos[:, :2].to(device=self.device, dtype=torch.float32)
        num_envs = pos_xy.shape[0]
        if turn_token is None and branch_ids is None:
            wp_xy = (
                self._waypoints_xy
                if self._waypoints_xy is not None
                else self.waypoints[:, :2]
            )
            dists = torch.cdist(pos_xy, wp_xy)
            return torch.argmin(dists, dim=1)

        if self.branch_waypoints_xy is None:
            self._build_multi_branch_bank()
        b_ids = self.resolve_branch_ids(
            num_envs, turn_token=turn_token, branch_ids=branch_ids
        )
        active_wp_xy = self.branch_waypoints_xy[b_ids]
        dists = torch.norm(pos_xy[:, None, :] - active_wp_xy, dim=2)
        return torch.argmin(dists, dim=1)

    def compute_marker_distances(self, pos: torch.Tensor):
        self.ensure_synced()
        pos_xy = pos[:, :2].to(device=self.device, dtype=torch.float32).contiguous()

        if (
            self._cached_marker_pos_xy is not None
            and self._cached_marker_pos_xy.shape == pos_xy.shape
            and torch.equal(self._cached_marker_pos_xy, pos_xy)
            and self._cached_dist_y is not None
            and self._cached_dist_w is not None
            and self._cached_dist_g is not None
        ):
            return self._cached_dist_y, self._cached_dist_w, self._cached_dist_g

        yellow_xy = (
            self._yellow_xy
            if self._yellow_xy is not None and self._yellow_xy.shape[0] == self.yellow_tensor.shape[0]
            else self.yellow_tensor[:, :2]
        )
        white_xy = (
            self._white_xy
            if self._white_xy is not None and self._white_xy.shape[0] == self.white_tensor.shape[0]
            else self.white_tensor[:, :2]
        )

        # 2D distance for XY plane, ignoring Z height of the robot chassis
        dist_y = torch.min(torch.cdist(pos_xy, yellow_xy), dim=1)[0]
        dist_w = torch.min(torch.cdist(pos_xy, white_xy), dim=1)[0]

        # Gates distance (permeable)
        if self.gate_tensor is not None:
            gate_xy = (
                self._gate_xy
                if self._gate_xy is not None and self._gate_xy.shape[0] == self.gate_tensor.shape[0]
                else self.gate_tensor[:, :2]
            )
            dist_g = torch.min(torch.cdist(pos_xy, gate_xy), dim=1)[0]
        else:
            dist_g = torch.ones_like(dist_y) * 10.0

        self._cached_marker_pos_xy = pos_xy.clone()
        self._cached_dist_y = dist_y
        self._cached_dist_w = dist_w
        self._cached_dist_g = dist_g
        return dist_y, dist_w, dist_g

    def refresh_visuals(self, target_lane_offset: float = 0.0):
        """Delegates debug visualization markers to track_visualizer."""
        refresh_track_visuals(self, target_lane_offset=target_lane_offset)

    def compute_errors(
        self,
        pos: torch.Tensor,
        yaw: torch.Tensor,
        target_lane_offset: float = 0.0,
        search_window: int = 50,
        turn_token: torch.Tensor | float | int | None = None,
        branch_ids: torch.Tensor | None = None,
    ):
        """Calculates signed lateral error, heading error, and curvature.

        Maintains robust continuity using local search windows around last_indices,
        and supports vectorized per-env branch indexing (Gate 1: branch 0=Left, 1=Straight, 2=Right).
        """
        self.ensure_synced(target_lane_offset=target_lane_offset)
        if self.branch_waypoints is None:
            self._build_multi_branch_bank()

        num_envs = pos.shape[0]
        num_wps = self.waypoints.shape[0]
        pos_xy = pos[:, :2].to(device=self.device, dtype=torch.float32)
        yaw_t = yaw.to(device=self.device, dtype=torch.float32).view(-1)

        b_ids = self.resolve_branch_ids(
            num_envs, turn_token=turn_token, branch_ids=branch_ids
        )
        if self.last_branch_ids is None or self.last_branch_ids.shape[0] != num_envs:
            switched = torch.zeros(num_envs, device=self.device, dtype=torch.bool)
        else:
            switched = b_ids != self.last_branch_ids
        self.branch_switched_mask = switched
        self.last_branch_ids = b_ids.clone()

        use_single_loop = bool((b_ids == 0).all().item()) and not bool(
            switched.any().item()
        )

        if use_single_loop:
            wp_xy = (
                self._waypoints_xy
                if self._waypoints_xy is not None
                and self._waypoints_xy.shape[0] == num_wps
                else self.waypoints[:, :2]
            )

            if self.last_indices is None or self.last_indices.shape[0] != num_envs:
                dists = torch.cdist(pos_xy, wp_xy)
                closest_idx = torch.argmin(dists, dim=1)
                self.last_indices = closest_idx
            else:
                offsets = torch.arange(
                    -search_window,
                    search_window + 1,
                    device=self.device,
                    dtype=torch.long,
                )
                candidate_indices = (
                    self.last_indices.unsqueeze(1) + offsets
                ) % num_wps

                candidate_wps = self.waypoints[candidate_indices]
                dists = torch.norm(
                    pos_xy[:, None, :] - candidate_wps[:, :, :2], dim=2
                )
                best_local_idx = torch.argmin(dists, dim=1)

                env_idx_range = torch.arange(num_envs, device=self.device)
                closest_idx = candidate_indices[env_idx_range, best_local_idx]

                min_local_dist = dists[env_idx_range, best_local_idx]
                teleported = min_local_dist > 1.0
                if teleported.any():
                    global_dists = torch.cdist(pos_xy, wp_xy)
                    global_idx = torch.argmin(global_dists, dim=1)
                    closest_idx = torch.where(teleported, global_idx, closest_idx)

                self.last_indices = closest_idx

            closest_wp = self.waypoints[closest_idx]
            kappa = self.curvature_tensor[closest_idx]
        else:
            env_idx_range = torch.arange(num_envs, device=self.device)
            active_wps = self.branch_waypoints[b_ids]
            active_wps_xy = self.branch_waypoints_xy[b_ids]

            if self.last_indices is None or self.last_indices.shape[0] != num_envs:
                dists = torch.norm(pos_xy[:, None, :] - active_wps_xy, dim=2)
                closest_idx = torch.argmin(dists, dim=1)
                self.last_indices = closest_idx
            else:
                offsets = torch.arange(
                    -search_window,
                    search_window + 1,
                    device=self.device,
                    dtype=torch.long,
                )
                raw_cand = self.last_indices.unsqueeze(1) + offsets.unsqueeze(0)
                seg_bounds = self.branch_seg_bounds[b_ids, self.last_indices]
                seg_min = seg_bounds[:, 0:1]
                seg_max = seg_bounds[:, 1:2]
                is_loop = (b_ids == 0).unsqueeze(1)
                candidate_indices = torch.where(
                    is_loop,
                    (raw_cand + num_wps) % num_wps,
                    torch.clamp(raw_cand, min=seg_min, max=seg_max),
                )

                candidate_wps = active_wps[
                    env_idx_range.unsqueeze(1), candidate_indices
                ]
                dists = torch.norm(
                    pos_xy[:, None, :] - candidate_wps[:, :, :2], dim=2
                )
                best_local_idx = torch.argmin(dists, dim=1)
                closest_idx = candidate_indices[env_idx_range, best_local_idx]

                min_local_dist = dists[env_idx_range, best_local_idx]
                need_global = (min_local_dist > 1.0) | switched
                if need_global.any():
                    global_dists = torch.norm(
                        pos_xy[:, None, :] - active_wps_xy, dim=2
                    )
                    global_idx = torch.argmin(global_dists, dim=1)
                    closest_idx = torch.where(need_global, global_idx, closest_idx)

                self.last_indices = closest_idx

            closest_wp = self.branch_waypoints[b_ids, closest_idx]
            kappa = self.branch_curvatures[b_ids, closest_idx]

        wp_to_robot = pos_xy - closest_wp[:, :2]
        wp_yaw = closest_wp[:, 2]

        normal_x, normal_y = torch.sin(wp_yaw), -torch.cos(wp_yaw)
        raw_lat_err = wp_to_robot[:, 0] * normal_x + wp_to_robot[:, 1] * normal_y

        # Vectorized offset application
        lat_err = raw_lat_err - target_lane_offset
        head_err = yaw_t - wp_yaw

        # Normalize to [-pi, pi]
        head_err = torch.atan2(torch.sin(head_err), torch.cos(head_err))

        return lat_err, head_err, kappa

    def get_lateral_heading_curvature_error(
        self,
        pos: torch.Tensor,
        yaw: torch.Tensor,
        target_lane_offset: float = 0.0,
        search_window: int = 50,
        turn_token: torch.Tensor | float | int | None = None,
        branch_ids: torch.Tensor | None = None,
    ):
        """Backward-compatible alias for compute_errors."""
        return self.compute_errors(
            pos,
            yaw,
            target_lane_offset=target_lane_offset,
            search_window=search_window,
            turn_token=turn_token,
            branch_ids=branch_ids,
        )

    def get_forward_waypoints(
        self,
        pos: torch.Tensor,
        yaw: torch.Tensor,
        offsets=[20, 40, 60, 80, 100],
        turn_token: torch.Tensor | float | int | None = None,
        branch_ids: torch.Tensor | None = None,
    ):
        """Extracts future waypoints relative to the robot's local frame."""
        if self.last_indices is None:
            return torch.zeros(pos.shape[0], len(offsets) * 2, device=self.device)

        self.ensure_synced()
        if self.branch_waypoints is None:
            self._build_multi_branch_bank()

        num_envs = pos.shape[0]
        num_wps = self.waypoints.shape[0]
        pos_xy = pos[:, :2].to(device=self.device, dtype=torch.float32)
        yaw_t = yaw.to(device=self.device, dtype=torch.float32).view(-1)
        offsets_t = torch.tensor(offsets, device=self.device, dtype=torch.long)

        if turn_token is None and branch_ids is None and (
            self.last_branch_ids is None or bool((self.last_branch_ids == 0).all().item())
        ):
            target_indices = (
                self.last_indices.view(-1, 1) + offsets_t
            ) % num_wps
            target_wps = self.waypoints[target_indices][:, :, :2]
        else:
            b_ids = (
                self.resolve_branch_ids(
                    num_envs, turn_token=turn_token, branch_ids=branch_ids
                )
                if (turn_token is not None or branch_ids is not None)
                else self.resolve_branch_ids(
                    num_envs, branch_ids=self.last_branch_ids
                )
            )
            raw_targets = self.last_indices.view(-1, 1) + offsets_t.unsqueeze(0)
            seg_bounds = self.branch_seg_bounds[b_ids, self.last_indices]
            seg_min = seg_bounds[:, 0:1]
            seg_max = seg_bounds[:, 1:2]
            is_loop = (b_ids == 0).unsqueeze(1)
            target_indices = torch.where(
                is_loop,
                (raw_targets + num_wps) % num_wps,
                torch.clamp(raw_targets, min=seg_min, max=seg_max),
            )
            target_wps = self.branch_waypoints[
                b_ids.unsqueeze(1), target_indices
            ][:, :, :2]

        rel_pos = target_wps - pos_xy[:, None, :]

        cos_y = torch.cos(-yaw_t)
        sin_y = torch.sin(-yaw_t)

        local_x = rel_pos[:, :, 0] * cos_y.unsqueeze(1) - rel_pos[
            :, :, 1
        ] * sin_y.unsqueeze(1)
        local_y = rel_pos[:, :, 0] * sin_y.unsqueeze(1) + rel_pos[
            :, :, 1
        ] * cos_y.unsqueeze(1)

        return torch.stack((local_x, local_y), dim=-1).view(num_envs, -1)

    def get_lookahead_curvature(
        self,
        lookahead_distance: float = 1.2,
        track_dir: torch.Tensor | None = None,
        turn_token: torch.Tensor | float | int | None = None,
        branch_ids: torch.Tensor | None = None,
    ) -> torch.Tensor:
        """Calculates lookahead track curvature in the vehicle's ego frame."""
        if (
            self.last_indices is None
            or self.curvature_tensor is None
            or self.waypoints is None
            or len(self.waypoints) == 0
        ):
            if track_dir is not None and isinstance(track_dir, torch.Tensor):
                batch_size = track_dir.shape[0] if track_dir.ndim > 0 else 1
            elif self.last_indices is not None:
                batch_size = (
                    self.last_indices.shape[0] if self.last_indices.ndim > 0 else 1
                )
            else:
                batch_size = 1
            return torch.zeros(batch_size, device=self.device, dtype=torch.float32)

        if self.last_indices.numel() == 0:
            return torch.empty(0, device=self.device, dtype=torch.float32)

        if self.branch_curvatures is None:
            self._build_multi_branch_bank()

        num_envs = self.last_indices.shape[0]
        num_wps = self.curvature_tensor.shape[0]

        if num_wps == 0:
            return torch.zeros(num_envs, device=self.device, dtype=torch.float32)

        step_offset = int(round(lookahead_distance / 0.010))

        if track_dir is None:
            dir_sign = torch.ones(num_envs, device=self.device, dtype=torch.long)
            dir_float = torch.ones(num_envs, device=self.device, dtype=torch.float32)
        elif isinstance(track_dir, torch.Tensor):
            td = track_dir.to(device=self.device, dtype=torch.float32)
            if td.ndim == 0:
                td = td.unsqueeze(0).expand(num_envs)
            elif td.ndim > 1:
                td = td.view(-1)
            if td.shape[0] != num_envs:
                td = (
                    td[:num_envs]
                    if td.shape[0] > num_envs
                    else torch.nn.functional.pad(
                        td, (0, num_envs - td.shape[0]), value=1.0
                    )
                )
            dir_sign = torch.where(
                td < 0.0,
                torch.tensor(-1, device=self.device, dtype=torch.long),
                torch.tensor(1, device=self.device, dtype=torch.long),
            )
            dir_float = dir_sign.to(dtype=torch.float32)
        else:
            val = -1 if float(track_dir) < 0 else 1
            dir_sign = torch.full(
                (num_envs,), val, device=self.device, dtype=torch.long
            )
            dir_float = dir_sign.to(dtype=torch.float32)

        fracs = torch.linspace(0.0, 1.0, steps=5, device=self.device)
        step_offsets = torch.round(fracs * step_offset).to(dtype=torch.long)

        base_indices = self.last_indices.to(
            device=self.device, dtype=torch.long
        ).unsqueeze(1)

        if turn_token is None and branch_ids is None and (
            self.last_branch_ids is None or bool((self.last_branch_ids == 0).all().item())
        ):
            window_indices = (
                base_indices + dir_sign.unsqueeze(1) * step_offsets.unsqueeze(0)
            ) % num_wps
            window_indices = (window_indices + num_wps) % num_wps
            kappa_window = self.curvature_tensor[window_indices].to(
                device=self.device, dtype=torch.float32
            ) * dir_float.unsqueeze(1)
        else:
            b_ids = (
                self.resolve_branch_ids(
                    num_envs, turn_token=turn_token, branch_ids=branch_ids
                )
                if (turn_token is not None or branch_ids is not None)
                else self.resolve_branch_ids(
                    num_envs, branch_ids=self.last_branch_ids
                )
            )
            raw_window = (
                base_indices + dir_sign.unsqueeze(1) * step_offsets.unsqueeze(0)
            )
            seg_bounds = self.branch_seg_bounds[b_ids, self.last_indices]
            seg_min = seg_bounds[:, 0:1]
            seg_max = seg_bounds[:, 1:2]
            is_loop = (b_ids == 0).unsqueeze(1)
            window_indices = torch.where(
                is_loop,
                ((raw_window % num_wps) + num_wps) % num_wps,
                torch.clamp(raw_window, min=seg_min, max=seg_max),
            )
            kappa_window = self.branch_curvatures[
                b_ids.unsqueeze(1), window_indices
            ].to(device=self.device, dtype=torch.float32) * dir_float.unsqueeze(1)

        max_idx = torch.argmax(torch.abs(kappa_window), dim=1, keepdim=True)
        signed_curvature = torch.gather(kappa_window, dim=1, index=max_idx).squeeze(1)

        signed_curvature = torch.nan_to_num(
            signed_curvature, nan=0.0, posinf=3.0, neginf=-3.0
        )
        clamped_curvature = torch.clamp(signed_curvature, min=-3.0, max=3.0)
        return clamped_curvature.to(device=self.device, dtype=torch.float32)


_TRACK_MANAGER = None


def get_track_manager(device: str | torch.device = "cuda:0"):
    global _TRACK_MANAGER
    if hasattr(device, "device"):
        device = device.device
    if _TRACK_MANAGER is None or str(_TRACK_MANAGER.device) != str(device):
        _TRACK_MANAGER = TrackManager(device=device)
    return _TRACK_MANAGER


sys.modules.setdefault("mdp.track_manager", sys.modules[__name__])
sys.modules.setdefault("arcproLab.mdp.track_manager", sys.modules[__name__])

