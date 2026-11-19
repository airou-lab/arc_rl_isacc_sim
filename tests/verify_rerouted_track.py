#!/usr/bin/env python3
"""Hostile Multi-Metric Track & Waypoint Verification Suite (Pure CPU)."""

import os
import sys

import numpy as np
import pytest
import torch
from scipy.spatial import cKDTree

ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ARCPRO_LAB_DIR = os.path.join(ROOT_DIR, "arcproLab")
for _p in (ROOT_DIR, ARCPRO_LAB_DIR):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from mdp.track_manager import TrackManager  # noqa: E402
from mdp.track_topology import (  # noqa: E402
    BOUNDARY_WARN_MARGIN,
    SECTOR_DEFS,
    SECTOR_DESCRIPTIONS,
)

pytestmark = pytest.mark.cpu


def verify():
    wp_file = os.path.join(ARCPRO_LAB_DIR, "mdp", "track_centerline.npy")
    bounds_file = os.path.join(ARCPRO_LAB_DIR, "mdp", "track_boundaries_1x.npz")

    # 1. Existence and Shape Gate
    assert os.path.exists(wp_file), f"FAIL: {wp_file} does not exist"
    assert os.path.exists(bounds_file), f"FAIL: {bounds_file} does not exist"

    wps = np.load(wp_file)
    assert wps.shape == (5000, 3), f"FAIL: Expected shape (5000, 3), got {wps.shape}"
    assert not np.isnan(wps).any(), "FAIL: NaNs found in waypoints"
    assert not np.isinf(wps).any(), "FAIL: Infs found in waypoints"

    # 2. Physical Asphalt & Boundary Coverage Gate
    bounds = np.load(bounds_file)
    yellow = bounds["yellow"][:, :2]
    white = bounds["white"][:, :2]
    tree_y = cKDTree(yellow)
    tree_w = cKDTree(white)

    d_y_all, _ = tree_y.query(wps[:, :2])
    d_w_all, _ = tree_w.query(wps[:, :2])

    assert np.min(d_y_all) >= BOUNDARY_WARN_MARGIN, (
        f"FAIL: Global minimum yellow distance {np.min(d_y_all):.4f}m < {BOUNDARY_WARN_MARGIN}m"
    )
    assert np.min(d_w_all) >= BOUNDARY_WARN_MARGIN, (
        f"FAIL: Global minimum white distance {np.min(d_w_all):.4f}m < {BOUNDARY_WARN_MARGIN}m"
    )
    assert np.mean(d_y_all) < 0.25, (
        f"FAIL: Mean distance to yellow too large: {np.mean(d_y_all):.3f}m"
    )
    assert np.mean(d_w_all) < 0.25, (
        f"FAIL: Mean distance to white too large: {np.mean(d_w_all):.3f}m"
    )

    # 3. Void Traversal Gate (Zero Points in the Void)
    max_x = np.max(wps[:, 0])
    assert max_x < -2.0, (
        f"FAIL: Waypoints enter void gap! Max X = {max_x:.3f}m (must be < -2.0m)"
    )

    # 4. Smoothness & Yaw Step Continuity Gate
    yaw = wps[:, 2]
    dyaw = np.diff(yaw, append=yaw[0])
    dyaw = (dyaw + np.pi) % (2 * np.pi) - np.pi
    max_yaw_step = np.max(np.abs(dyaw))
    assert max_yaw_step < 0.03, f"FAIL: Yaw step too large: {max_yaw_step:.4f} rad"

    # 5. Loop Closure Gate
    closure_err = np.linalg.norm(wps[0, :2] - wps[-1, :2])
    assert closure_err < 0.03, (
        f"FAIL: Loop closure error too large: {closure_err:.4f} m"
    )

    # 6. Curriculum Sectors Alignment Gate using canonical SECTOR_DEFS
    for sec_id, (x, y, phys_yaw) in SECTOR_DEFS.items():
        name = SECTOR_DESCRIPTIONS[str(sec_id)]
        pos = np.array([x, y])
        robot_obs_yaw = (phys_yaw + np.pi + np.pi) % (2 * np.pi) - np.pi

        dists = np.linalg.norm(wps[:, :2] - pos, axis=1)
        closest_idx = np.argmin(dists)
        pos_err = dists[closest_idx]
        closest_wp = wps[closest_idx]
        head_err = (robot_obs_yaw - closest_wp[2] + np.pi) % (2 * np.pi) - np.pi

        assert pos_err < 0.05, f"FAIL: {name} pos error {pos_err:.4f}m > 0.05m"
        assert abs(head_err) < 0.05, (
            f"FAIL: {name} head error {head_err:.4f} rad > 0.05 rad"
        )

    # 7. TrackManager Integration Gate
    tm = TrackManager(device="cpu")
    tm.ensure_synced()
    assert tm.waypoints is not None and len(tm.waypoints) == 5000
    assert tm.curvature_tensor is not None and len(tm.curvature_tensor) == 5000
    assert not torch.isnan(tm.curvature_tensor).any()

    test_pos = torch.tensor([[-16.197, 5.50]])
    test_yaw = torch.tensor([1.5708])  # Heading South in observations frame
    lat_err, head_err, kappa = tm.compute_errors(test_pos, test_yaw)
    assert abs(lat_err.item()) < 0.05
    assert abs(head_err.item()) < 0.05


def test_verify_rerouted_track():
    verify()


if __name__ == "__main__":
    verify()
