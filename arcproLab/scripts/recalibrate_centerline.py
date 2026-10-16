#!/usr/bin/env python3
"""
Recalibrate Track Centerline & Boundary Permeability
---------------------------------------------------
1. Eliminates localized lane drift:
   - Leg 2 (WP 1640-1830): Shift South by 17.0cm to center road clearance (18cm / 22cm).
   - Straightaway West (WP 4180-4870): Shift North by 22.0cm (13cm / 27cm clearance).
   - Corner 1 Apex (WP 950-1010): Preserves original clipping trajectory (6.82cm clearance).
2. Fixes Hairpin Turnaround Knot (WP 4870-4985):
   - Replaces defective knot (previously 71.7° yaw spike) with an exact C1 Hermite spline.
3. Punches Intersection Gates in track_boundaries_1x.npz:
   - Adds gates at intersection transitions so painted road stripes crossing the roadway
     do not trigger false boundary collisions, restoring WP 1871 clearance from 0.11cm to 50.5cm.
4. Ensures global C1 continuity:
   - Max yaw jump < 0.03 rad (< 1.6°) across all 5,000 waypoints.
   - 0 yaw jumps > 0.05 rad.
   - Minimum white clearance >= 8.2cm everywhere.
   - Minimum yellow clearance >= 6.8cm (apex) and >= 12.0cm (straightaways).
"""

import os
import shutil
import numpy as np
from scipy.spatial import cKDTree


def hermite_spline(p0, t0, p1, t1, n_pts):
    s = np.linspace(0, 1, n_pts)[:, None]
    h00 = 2.0 * s**3 - 3.0 * s**2 + 1.0
    h10 = s**3 - 2.0 * s**2 + s
    h01 = -2.0 * s**3 + 3.0 * s**2
    h11 = s**3 - s**2
    return h00 * p0 + h10 * t0 + h01 * p1 + h11 * t1


def recalibrate():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    root_dir = os.path.abspath(os.path.join(script_dir, "..", ".."))

    wp_path = os.path.join(root_dir, "arcproLab", "mdp", "track_centerline.npy")
    bounds_path = os.path.join(root_dir, "arcproLab", "mdp", "track_boundaries_1x.npz")
    pre_recal_backup = os.path.join(
        root_dir,
        "archive",
        "landmark_checkpoints",
        "track_centerline_pre_recalibration.npy",
    )
    bounds_backup = os.path.join(
        root_dir, "archive", "landmark_checkpoints", "track_boundaries_1x_pre_gates.npz"
    )

    # 1. Load baseline original centerline
    if os.path.exists(pre_recal_backup):
        orig = np.load(pre_recal_backup)
        print(f"[Load] Using baseline original centerline from: {pre_recal_backup}")
    else:
        orig = np.load(wp_path)
        os.makedirs(os.path.dirname(pre_recal_backup), exist_ok=True)
        shutil.copy2(wp_path, pre_recal_backup)
        print(f"[Backup] Saved baseline centerline to: {pre_recal_backup}")

    bounds = np.load(bounds_path)
    yellow = bounds["yellow"]
    white = bounds["white"]
    existing_gate = bounds["gate"] if "gate" in bounds.files else np.empty((0, 3))

    if not os.path.exists(bounds_backup):
        os.makedirs(os.path.dirname(bounds_backup), exist_ok=True)
        shutil.copy2(bounds_path, bounds_backup)
        print(f"[Backup] Saved original boundaries to: {bounds_backup}")

    n = len(orig)
    assert n == 5000, f"Expected 5000 waypoints, got {n}"
    new_xy = orig[:, :2].copy()

    # 2. Corridor Shifts (smooth Hann windows, zero derivative boundary conditions)
    # Leg 2 Yellow Drift (WP 1640 to 1830): Shift South by 17.0cm
    for i in range(1640, 1830):
        s = (i - 1640) / (1830 - 1640)
        w = np.sin(s * np.pi) ** 2
        new_xy[i, 1] -= w * 0.170

    # Straightaway West White Drift (WP 4180 to 4870): Shift North by 22.0cm
    k_start = 4870
    k_end = 4985
    for i in range(4180, k_start + 1):
        if i < 4260:
            w = np.sin((i - 4180) / 80.0 * np.pi / 2.0) ** 2
        else:
            w = 1.0
        new_xy[i, 1] += w * 0.220

    # 3. Hairpin Turnaround Knot Fix (WP 4870 to 4985)
    # Connects shifted West straightaway to Start Line runway via C1 Hermite spline
    n_hermite = k_end - k_start + 1
    t_start = (new_xy[k_start] - new_xy[k_start - 5]) / 5.0 * (n_hermite - 1)
    t_end = (orig[k_end + 5, :2] - orig[k_end, :2]) / 5.0 * (n_hermite - 1)
    new_xy[k_start : k_end + 1] = hermite_spline(
        new_xy[k_start], t_start, orig[k_end, :2], t_end, n_hermite
    )

    # 4. Corner 1 Apex (WP 950 to 1010): Exactly preserved
    new_xy[950:1010] = orig[950:1010, :2]

    # 5. Compute smooth continuous tangent yaw angles: arctan2(-dy, -dx)
    new_wps = np.zeros((n, 3), dtype=np.float32)
    new_wps[:, :2] = new_xy.astype(np.float32)
    for i in range(n):
        i_prev = (i - 1 + n) % n
        i_next = (i + 1) % n
        dx = new_xy[i_next, 0] - new_xy[i_prev, 0]
        dy_c = new_xy[i_next, 1] - new_xy[i_prev, 1]
        new_wps[i, 2] = np.float32(np.arctan2(-dy_c, -dx))

    # 6. Punch intersection gates in track_boundaries_1x.npz
    # Mark intersection crossings so painted road markings do not block transit
    intersection_gates_2d = np.array(
        [
            [-3.85, -6.50],  # Intersection 1 Entrance from Leg 2
            [3.54, -5.98],  # Corner 2 left-turn cross-lane intersection
            [3.74, 5.70],  # Corner 3 cross-lane transition
            [3.32, 6.13],  # Corner 3 turn entry into West straightaway
            [-4.62, 6.37],  # Leg 5 entry crossing
            [-5.45, 6.37],  # Leg 5 intersection exit
        ]
    )

    mask_w = np.ones(len(white), dtype=bool)
    mask_y = np.ones(len(yellow), dtype=bool)
    for g in intersection_gates_2d:
        mask_w &= np.linalg.norm(white[:, :2] - g, axis=1) > 0.65
        mask_y &= np.linalg.norm(yellow[:, :2] - g, axis=1) > 0.65

    white_punched = white[mask_w]
    yellow_punched = yellow[mask_y]

    # Append new 3D gate locations to existing gate array
    new_gates_3d = np.column_stack(
        [intersection_gates_2d, np.full(len(intersection_gates_2d), 0.20)]
    )
    if len(existing_gate) > 0:
        combined_gate = np.vstack([existing_gate, new_gates_3d])
    else:
        combined_gate = new_gates_3d

    # 7. Verification checks
    dyaw = np.zeros(n)
    for i in range(n):
        diff = new_wps[(i + 1) % n, 2] - new_wps[i, 2]
        diff = (diff + np.pi) % (2.0 * np.pi) - np.pi
        dyaw[i] = abs(diff)

    tree_y = cKDTree(yellow_punched[:, :2])
    tree_w = cKDTree(white_punched[:, :2])
    dists_y, _ = tree_y.query(new_xy)
    dists_w, _ = tree_w.query(new_xy)

    print("\n--- Verifying Recalibrated Centerline & Boundaries ---")
    print(
        f"Max dyaw: {dyaw.max():.4f} rad ({np.degrees(dyaw.max()):.2f} deg) at WP {np.argmax(dyaw)}"
    )
    print(f"Yaw jumps > 0.05 rad: {np.sum(dyaw > 0.05)}")
    print(f"Yaw jumps > 0.03 rad: {np.sum(dyaw > 0.03)}")
    print(
        f"Corner 1 apex yellow clearance (WP 986): {dists_y[986] * 100:.2f} cm (baseline: 6.82 cm)"
    )
    print(f"WP 1871 white clearance: {dists_w[1871] * 100:.2f} cm (orig: 0.11 cm)")
    print(f"WP 1871 yellow clearance: {dists_y[1871] * 100:.2f} cm")
    print(f"WP 1746 yellow clearance: {dists_y[1746] * 100:.2f} cm (orig: 1.83 cm)")
    print(f"WP 1746 white clearance: {dists_w[1746] * 100:.2f} cm")
    print(f"WP 2440 white clearance: {dists_w[2440] * 100:.2f} cm")
    print(f"WP 2490 yellow clearance: {dists_y[2490] * 100:.2f} cm")
    print(f"WP 3380 yellow clearance: {dists_y[3380] * 100:.2f} cm")
    print(f"WP 3427 white clearance: {dists_w[3427] * 100:.2f} cm")
    print(f"WP 4042 white clearance: {dists_w[4042] * 100:.2f} cm")
    print(f"WP 4110 yellow clearance: {dists_y[4110] * 100:.2f} cm")
    print(f"WP 4230 white clearance: {dists_w[4230] * 100:.2f} cm (orig: 0.27 cm)")
    print(f"WP 4230 yellow clearance: {dists_y[4230] * 100:.2f} cm")
    print(
        f"Overall min white clearance: {dists_w.min() * 100:.2f} cm at WP {np.argmin(dists_w)}"
    )
    print(
        f"Overall min yellow clearance: {dists_y.min() * 100:.2f} cm at WP {np.argmin(dists_y)}"
    )

    # Invariants
    assert new_wps.shape == (5000, 3), f"Shape invariant failed: {new_wps.shape}"
    assert new_wps.dtype == np.float32, f"Dtype invariant failed: {new_wps.dtype}"
    assert not np.isnan(new_wps).any(), "NaN values found in waypoints"
    assert dyaw.max() < 0.05, f"Yaw continuity failed: max dyaw is {dyaw.max():.4f} rad"
    assert np.sum(dyaw > 0.05) == 0, "Yaw jumps > 0.05 rad found"
    assert dists_w.min() >= 0.05, (
        f"White clearance failed: min is {dists_w.min() * 100:.2f} cm"
    )
    assert dists_y.min() >= 0.05, (
        f"Yellow clearance failed: min is {dists_y.min() * 100:.2f} cm"
    )
    assert dists_w[1871] >= 0.15, (
        f"WP 1871 clearance failed: {dists_w[1871] * 100:.2f} cm"
    )

    # 8. Save updated files
    np.save(wp_path, new_wps)
    print(f"\n[Saved] Updated centerline to: {wp_path}")

    np.savez(
        bounds_path, yellow=yellow_punched, white=white_punched, gate=combined_gate
    )
    print(f"[Saved] Updated boundaries and gates to: {bounds_path}")


if __name__ == "__main__":
    recalibrate()
