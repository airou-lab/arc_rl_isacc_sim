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
import numpy as np
import torch

from .track_usd_extractor import (
    collect_raw_marker_points_from_stage,
    finalize_group as _finalize_group,
    get_mesh_info as _get_mesh_info,
    punch_holes as _punch_holes,
    sample_waypoints as _sample_waypoints,
)
from .track_visualizer import refresh_track_visuals


class TrackManager:
    """Robust Track Manager with VisualizationMarkers and Gap-Filling.

    Ensures 'Dense Walls' for reliable termination and visible debug spheres.
    """

    def __init__(self, device: str = "cuda:0"):
        self.device = device
        self.waypoints = None
        self.last_indices = None  # Tensor (num_envs,) to track closest waypoints

        # Performance: Only enable visuals if explicitly requested via --debug flag
        import sys

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

            # Convert to Tensors
            if self.raw_yellow_pts is not None:
                self.yellow_tensor = torch.tensor(
                    self.raw_yellow_pts, device=self.device, dtype=torch.float32
                )
            if self.raw_white_pts is not None:
                self.white_tensor = torch.tensor(
                    self.raw_white_pts, device=self.device, dtype=torch.float32
                )
            if self.raw_gate_pts is not None:
                self.gate_tensor = torch.tensor(
                    self.raw_gate_pts, device=self.device, dtype=torch.float32
                )

            if os.path.exists(self.wp_path):
                wp_np = np.load(self.wp_path)
                self.waypoints = torch.tensor(
                    wp_np, device=self.device, dtype=torch.float32
                )

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
                self.curvature_tensor = torch.zeros(1, device=self.device)

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
            y = (
                self.raw_yellow_pts
                if self.raw_yellow_pts is not None
                else np.array([])
            )
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
        print(f"[TrackManager] Generated {n} sequenced waypoints.")

    def save_waypoints(self, path):
        if self.waypoints is not None:
            np.save(path, self.waypoints.cpu().numpy())
            print(f"[TrackManager] Saved waypoints to {path}")

    def compute_marker_distances(self, pos: torch.Tensor):
        self.ensure_synced()
        # 2D distance for XY plane, ignoring Z height of the robot chassis
        dist_y = torch.min(torch.cdist(pos[:, :2], self.yellow_tensor[:, :2]), dim=1)[0]
        dist_w = torch.min(torch.cdist(pos[:, :2], self.white_tensor[:, :2]), dim=1)[0]

        # Gates distance (permeable)
        if self.gate_tensor is not None:
            dist_g = torch.min(
                torch.cdist(pos[:, :2], self.gate_tensor[:, :2]), dim=1
            )[0]
        else:
            dist_g = torch.ones_like(dist_y) * 10.0

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
    ):
        """Calculates signed lateral error, heading error, and curvature.

        Maintains robust continuity using local search windows around last_indices.
        """
        self.ensure_synced(target_lane_offset=target_lane_offset)

        num_envs = pos.shape[0]
        num_wps = self.waypoints.shape[0]

        if self.last_indices is None or self.last_indices.shape[0] != num_envs:
            # Cold start: Global Search
            dists = torch.cdist(pos[:, :2], self.waypoints[:, :2])
            closest_idx = torch.argmin(dists, dim=1)
            self.last_indices = closest_idx
        else:
            # Local Window Search to prevent teleportation
            offsets = torch.arange(
                -search_window,
                search_window + 1,
                device=self.device,
                dtype=torch.long,
            )
            candidate_indices = (self.last_indices.unsqueeze(1) + offsets) % num_wps

            candidate_wps = self.waypoints[candidate_indices]
            dists = torch.norm(pos[:, None, :2] - candidate_wps[:, :, :2], dim=2)
            best_local_idx = torch.argmin(dists, dim=1)

            closest_idx = candidate_indices[
                torch.arange(num_envs, device=self.device), best_local_idx
            ]

            # Teleport / Spawn detection: if closest waypoint in local window is > 1.0m away,
            # fall back to global search to re-index at new spawn location
            min_local_dist = dists[
                torch.arange(num_envs, device=self.device), best_local_idx
            ]
            teleported = min_local_dist > 1.0
            if teleported.any():
                global_dists = torch.cdist(pos[:, :2], self.waypoints[:, :2])
                global_idx = torch.argmin(global_dists, dim=1)
                closest_idx = torch.where(teleported, global_idx, closest_idx)

            self.last_indices = closest_idx

        closest_wp = self.waypoints[closest_idx]
        wp_to_robot = pos[:, :2] - closest_wp[:, :2]
        wp_yaw = closest_wp[:, 2]

        kappa = self.curvature_tensor[closest_idx]
        normal_x, normal_y = torch.sin(wp_yaw), -torch.cos(wp_yaw)
        raw_lat_err = wp_to_robot[:, 0] * normal_x + wp_to_robot[:, 1] * normal_y

        # Vectorized offset application
        lat_err = raw_lat_err - target_lane_offset
        head_err = yaw - wp_yaw

        # Normalize to [-pi, pi]
        head_err = torch.atan2(torch.sin(head_err), torch.cos(head_err))

        return lat_err, head_err, kappa

    def get_lateral_heading_curvature_error(
        self,
        pos: torch.Tensor,
        yaw: torch.Tensor,
        target_lane_offset: float = 0.0,
        search_window: int = 50,
    ):
        """Backward-compatible alias for compute_errors."""
        return self.compute_errors(
            pos, yaw, target_lane_offset=target_lane_offset, search_window=search_window
        )

    def get_forward_waypoints(
        self,
        pos: torch.Tensor,
        yaw: torch.Tensor,
        offsets=[20, 40, 60, 80, 100],
    ):
        """Extracts future waypoints relative to the robot's local frame."""
        if self.last_indices is None:
            return torch.zeros(pos.shape[0], len(offsets) * 2, device=self.device)

        num_envs = pos.shape[0]
        num_wps = self.waypoints.shape[0]

        target_indices = (
            self.last_indices.view(-1, 1) + torch.tensor(offsets, device=self.device)
        ) % num_wps
        target_wps = self.waypoints[target_indices][:, :, :2]

        rel_pos = target_wps - pos[:, None, :2]

        cos_y = torch.cos(-yaw)
        sin_y = torch.sin(-yaw)

        local_x = (
            rel_pos[:, :, 0] * cos_y.unsqueeze(1) - rel_pos[:, :, 1] * sin_y.unsqueeze(1)
        )
        local_y = (
            rel_pos[:, :, 0] * sin_y.unsqueeze(1) + rel_pos[:, :, 1] * cos_y.unsqueeze(1)
        )

        return torch.stack((local_x, local_y), dim=-1).view(num_envs, -1)

    def get_lookahead_curvature(
        self,
        lookahead_distance: float = 1.2,
        track_dir: torch.Tensor | None = None,
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
            dir_sign = torch.full((num_envs,), val, device=self.device, dtype=torch.long)
            dir_float = dir_sign.to(dtype=torch.float32)

        lookahead_indices = (
            self.last_indices.to(device=self.device, dtype=torch.long)
            + dir_sign * step_offset
        ) % num_wps
        lookahead_indices = (lookahead_indices + num_wps) % num_wps

        kappa_wp = self.curvature_tensor[lookahead_indices].to(
            device=self.device, dtype=torch.float32
        )
        signed_curvature = kappa_wp * dir_float

        signed_curvature = torch.nan_to_num(
            signed_curvature, nan=0.0, posinf=3.0, neginf=-3.0
        )
        clamped_curvature = torch.clamp(signed_curvature, min=-3.0, max=3.0)
        return clamped_curvature.to(device=self.device, dtype=torch.float32)


_TRACK_MANAGER = None


def get_track_manager(device: str = "cuda:0"):
    global _TRACK_MANAGER
    if _TRACK_MANAGER is None:
        _TRACK_MANAGER = TrackManager(device=device)
    return _TRACK_MANAGER
