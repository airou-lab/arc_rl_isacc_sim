#!/usr/bin/env python3
"""
Invasive Full-Circuit Track Geometry & Boundary Clearance Auditor
Task: T-4.3.2-GEOM
Audits:
  - arcproLab/mdp/track_centerline.npy (5,000 waypoints)
  - arcproLab/mdp/track_boundaries_1x.npz (yellow, white, gate markers)

Computes:
1. Distance to inner yellow line (min, mean, max, std) & tightest bottleneck.
2. Distance to outer white line (min, mean, max, std) & tightest bottleneck.
3. Track width along the track (dist_y + dist_w), pinch points, and asymmetrical drifts.
4. Line clipping, unfair boundary triggers, and vehicle wheel margin analysis.
5. Exact coordinates (X, Y, Yaw, WP index) for Entry, Apex, and Exit across:
   - Corner 1
   - Corner 2 (Intersection)
   - Corner 3 (Bend)
   - Corner 4
"""

import os
import numpy as np
from scipy.spatial import cKDTree


def run_geometry_audit():
    print("=" * 80)
    print("     CLEARSET SWARM RUN: T-4.3.2-GEOM TRACK GEOMETRY AUDIT")
    print("=" * 80)

    centerline_path = "arcproLab/mdp/track_centerline.npy"
    boundaries_path = "arcproLab/mdp/track_boundaries_1x.npz"

    if not os.path.exists(centerline_path):
        raise FileNotFoundError(f"Missing centerline file: {centerline_path}")
    if not os.path.exists(boundaries_path):
        raise FileNotFoundError(f"Missing boundaries file: {boundaries_path}")

    wps = np.load(centerline_path)
    bounds = np.load(boundaries_path)

    print(f"\n[1] File Integrity & Structure:")
    print(f"  Centerline Waypoints Shape : {wps.shape} (dtype: {wps.dtype})")
    print(f"  Boundaries Keys            : {list(bounds.keys())}")
    for k in bounds.keys():
        print(f"    - '{k}' Shape            : {bounds[k].shape}")

    assert wps.shape == (5000, 3), f"Expected (5000, 3), got {wps.shape}"
    assert not np.isnan(wps).any(), "NaN found in waypoints!"
    assert not np.isinf(wps).any(), "Inf found in waypoints!"

    yellow = bounds["yellow"][:, :2]
    white = bounds["white"][:, :2]
    gate = bounds["gate"][:, :2] if "gate" in bounds else np.empty((0, 2))

    tree_y = cKDTree(yellow)
    tree_w = cKDTree(white)
    tree_g = cKDTree(gate) if len(gate) > 0 else None

    # Step distance & total circuit length
    diffs = np.diff(wps[:, :2], axis=0, append=wps[:1, :2])
    step_dists = np.linalg.norm(diffs, axis=1)
    cum_dists = np.insert(np.cumsum(step_dists), 0, 0.0)[:-1]
    total_length = np.sum(step_dists)

    # Curvature calculation (Kappa = dYaw / ds)
    n_wps = len(wps)
    kappa = np.zeros(n_wps)
    for i in range(n_wps):
        i1 = (i + 1) % n_wps
        dyaw = wps[i1, 2] - wps[i, 2]
        while dyaw > np.pi:
            dyaw -= 2 * np.pi
        while dyaw < -np.pi:
            dyaw += 2 * np.pi
        ds = step_dists[i]
        if ds > 0:
            kappa[i] = dyaw / ds

    # Nearest boundary distances for each waypoint
    dist_y, idx_y = tree_y.query(wps[:, :2])
    dist_w, idx_w = tree_w.query(wps[:, :2])
    dist_g, idx_g = tree_g.query(wps[:, :2]) if tree_g else (np.full(n_wps, 10.0), None)

    track_width = dist_y + dist_w
    lane_asymmetry = dist_y - dist_w  # >0: closer to white, <0: closer to yellow

    print(f"\n[2] Circuit Overview Metrics:")
    print(f"  Total Circuit Length       : {total_length:.3f} m")
    print(
        f"  Mean Waypoint Step (ds)    : {np.mean(step_dists) * 100:.3f} cm (min: {np.min(step_dists) * 100:.3f} cm, max: {np.max(step_dists) * 100:.3f} cm)"
    )
    print(
        f"  X Range                    : [{wps[:, 0].min():.3f}, {wps[:, 0].max():.3f}] m"
    )
    print(
        f"  Y Range                    : [{wps[:, 1].min():.3f}, {wps[:, 1].max():.3f}] m"
    )

    # -------------------------------------------------------------
    # 1. Yellow Line Metrics & Bottlenecks
    # -------------------------------------------------------------
    min_y_idx = np.argmin(dist_y)
    max_y_idx = np.argmax(dist_y)
    print(f"\n[3] Distance to Inner Yellow Line:")
    print(f"  Minimum Distance           : {dist_y.min():.4f} m (WP #{min_y_idx})")
    print(f"  Mean Distance              : {dist_y.mean():.4f} m")
    print(f"  Maximum Distance           : {dist_y.max():.4f} m (WP #{max_y_idx})")
    print(f"  Standard Deviation         : {dist_y.std():.4f} m")
    print(f"  Tightest Yellow Bottleneck :")
    print(f"    WP Index = {min_y_idx}")
    print(f"    Coord    = ({wps[min_y_idx, 0]:.4f}, {wps[min_y_idx, 1]:.4f})")
    print(
        f"    Yaw      = {wps[min_y_idx, 2]:.4f} rad ({np.degrees(wps[min_y_idx, 2]):.2f}°)"
    )
    print(
        f"    dist_y   = {dist_y[min_y_idx]:.4f} m ({dist_y[min_y_idx] * 100:.2f} cm)"
    )
    print(
        f"    dist_w   = {dist_w[min_y_idx]:.4f} m ({dist_w[min_y_idx] * 100:.2f} cm)"
    )
    print(
        f"    width    = {track_width[min_y_idx]:.4f} m ({track_width[min_y_idx] * 100:.2f} cm)"
    )

    # -------------------------------------------------------------
    # 2. White Line Metrics & Bottlenecks
    # -------------------------------------------------------------
    min_w_idx = np.argmin(dist_w)
    max_w_idx = np.argmax(dist_w)
    print(f"\n[4] Distance to Outer White Line:")
    print(f"  Minimum Distance           : {dist_w.min():.4f} m (WP #{min_w_idx})")
    print(f"  Mean Distance              : {dist_w.mean():.4f} m")
    print(f"  Maximum Distance           : {dist_w.max():.4f} m (WP #{max_w_idx})")
    print(f"  Standard Deviation         : {dist_w.std():.4f} m")
    print(f"  Tightest White Bottleneck  :")
    print(f"    WP Index = {min_w_idx}")
    print(f"    Coord    = ({wps[min_w_idx, 0]:.4f}, {wps[min_w_idx, 1]:.4f})")
    print(
        f"    Yaw      = {wps[min_w_idx, 2]:.4f} rad ({np.degrees(wps[min_w_idx, 2]):.2f}°)"
    )
    print(
        f"    dist_y   = {dist_y[min_w_idx]:.4f} m ({dist_y[min_w_idx] * 100:.2f} cm)"
    )
    print(
        f"    dist_w   = {dist_w[min_w_idx]:.4f} m ({dist_w[min_w_idx] * 100:.2f} cm)"
    )
    print(
        f"    width    = {track_width[min_w_idx]:.4f} m ({track_width[min_w_idx] * 100:.2f} cm)"
    )

    # -------------------------------------------------------------
    # 3. Track Width & Asymmetry Analysis
    # -------------------------------------------------------------
    min_width_idx = np.argmin(track_width)
    max_width_idx = np.argmax(track_width)
    print(f"\n[5] Track Width (dist_y + dist_w) & Symmetry:")
    print(
        f"  Minimum Width (Pinch Point): {track_width.min():.4f} m (WP #{min_width_idx})"
    )
    print(f"  Mean Width                 : {track_width.mean():.4f} m")
    print(
        f"  Maximum Width              : {track_width.max():.4f} m (WP #{max_width_idx})"
    )
    print(f"  Width Standard Deviation   : {track_width.std():.4f} m")
    print(f"  Tightest Pinch Point       :")
    print(f"    WP Index = {min_width_idx}")
    print(f"    Coord    = ({wps[min_width_idx, 0]:.4f}, {wps[min_width_idx, 1]:.4f})")
    print(
        f"    Yaw      = {wps[min_width_idx, 2]:.4f} rad ({np.degrees(wps[min_width_idx, 2]):.2f}°)"
    )
    print(
        f"    dist_y   = {dist_y[min_width_idx]:.4f} m, dist_w = {dist_w[min_width_idx]:.4f} m, width = {track_width[min_width_idx]:.4f} m"
    )

    print(f"\n  Lane Centering Asymmetry (dist_y - dist_w):")
    print(
        f"    Mean |dist_y - dist_w|   : {np.mean(np.abs(lane_asymmetry)):.4f} m ({np.mean(np.abs(lane_asymmetry)) * 100:.2f} cm)"
    )
    print(
        f"    Max Drift toward White   : {lane_asymmetry.max():.4f} m (WP #{np.argmax(lane_asymmetry)}) at ({wps[np.argmax(lane_asymmetry), 0]:.3f}, {wps[np.argmax(lane_asymmetry), 1]:.3f})"
    )
    print(
        f"    Max Drift toward Yellow  : {abs(lane_asymmetry.min()):.4f} m (WP #{np.argmin(lane_asymmetry)}) at ({wps[np.argmin(lane_asymmetry), 0]:.3f}, {wps[np.argmin(lane_asymmetry), 1]:.3f})"
    )
    asym_gt_1cm = np.sum(np.abs(lane_asymmetry) > 0.01)
    asym_gt_2cm = np.sum(np.abs(lane_asymmetry) > 0.02)
    asym_gt_5cm = np.sum(np.abs(lane_asymmetry) > 0.05)
    print(
        f"    Waypoints with |asym| > 1cm: {asym_gt_1cm} / 5000 ({asym_gt_1cm / 50:.1f}%)"
    )
    print(
        f"    Waypoints with |asym| > 2cm: {asym_gt_2cm} / 5000 ({asym_gt_2cm / 50:.1f}%)"
    )
    print(
        f"    Waypoints with |asym| > 5cm: {asym_gt_5cm} / 5000 ({asym_gt_5cm / 50:.1f}%)"
    )

    # -------------------------------------------------------------
    # 4. Clipping, Unfair Boundary Triggers & Wheel Margins
    # -------------------------------------------------------------
    print(f"\n[6] Invasive Boundary Triggers & Physical Vehicle Clearance Audit:")
    # Vehicle parameters
    # ARCPro chassis width: ~0.32m, track width (tire center-to-center): ~0.28m (half-track: 0.14m)
    # Tire outer edge = half-track + half_tire_width (~0.14 + 0.02 = 0.16m)
    tire_dist_y = dist_y - 0.14
    tire_dist_w = dist_w - 0.14

    # Terminations thresholds from terminations.py:
    # is_corner: |kappa| > 0.20
    # dyn_threshold_w: 0.015 (corner), 0.018 (straight)
    # dyn_threshold_y: 0.008 (corner), 0.020 (straight)
    is_corner_arr = np.abs(kappa) > 0.20
    term_thresh_w = np.where(is_corner_arr, 0.015, 0.018)
    term_thresh_y = np.where(is_corner_arr, 0.008, 0.020)

    # Permissive zone
    in_permissive = (
        (wps[:, 0] >= -7.0)
        & (wps[:, 0] <= -2.15)
        & (wps[:, 1] >= -7.5)
        & (wps[:, 1] <= -2.4)
    ) | (dist_g < 0.60)
    east_cliff = (wps[:, 0] > -2.08) & (wps[:, 1] >= -7.5) & (wps[:, 1] <= -2.4)

    # Check for direct centerline line clips
    clip_yellow_wps = np.where(dist_y < 0.05)[0]
    clip_white_wps = np.where(dist_w < 0.05)[0]
    print(f"  Centerline Waypoints with dist_y < 5cm : {len(clip_yellow_wps)}")
    print(f"  Centerline Waypoints with dist_w < 5cm : {len(clip_white_wps)}")

    # Check for false termination triggers on centerline
    direct_term_y = (~in_permissive) & (dist_y < term_thresh_y)
    direct_term_w = (~in_permissive) & (dist_w < term_thresh_w)
    direct_term = direct_term_y | direct_term_w | east_cliff
    print(f"  Centerline Waypoints triggering false reset: {np.sum(direct_term)}")
    print(f"  Centerline Waypoints violating East Cliff  : {np.sum(east_cliff)}")

    # Warning thresholds in rewards.py: 0.18m
    warn_y_wps = np.where(dist_y < 0.18)[0]
    warn_w_wps = np.where(dist_w < 0.18)[0]
    print(f"  Waypoints with dist_y < 0.18m (Reward Warning Zone): {len(warn_y_wps)}")
    print(f"  Waypoints with dist_w < 0.18m (Reward Warning Zone): {len(warn_w_wps)}")
    if len(warn_y_wps) > 0:
        print(
            f"    - Yellow warn range: min dist_y={dist_y[warn_y_wps].min():.4f}m at WP #{warn_y_wps[np.argmin(dist_y[warn_y_wps])]}"
        )
    if len(warn_w_wps) > 0:
        print(
            f"    - White warn range:  min dist_w={dist_w[warn_w_wps].min():.4f}m at WP #{warn_w_wps[np.argmin(dist_w[warn_w_wps])]}"
        )

    # Physical tire margins when perfectly centered
    print(f"  Tire-to-Line Clearance (Centerline, Half-Track 14cm):")
    print(
        f"    Min Yellow Tire Clearance: {tire_dist_y.min() * 100:.2f} cm (WP #{np.argmin(tire_dist_y)})"
    )
    print(f"    Mean Yellow Tire Clearance: {tire_dist_y.mean() * 100:.2f} cm")
    print(
        f"    Min White Tire Clearance:  {tire_dist_w.min() * 100:.2f} cm (WP #{np.argmin(tire_dist_w)})"
    )
    print(f"    Mean White Tire Clearance:  {tire_dist_w.mean() * 100:.2f} cm")

    # -------------------------------------------------------------
    # 5. Corner Entry, Apex, and Exit Coordinates
    # -------------------------------------------------------------
    # Circuit structure:
    # Starting from WP 0:
    # WP 0 is near West runway (Y ~ 6.6, heading South)
    # Let's map waypoints along circuit:
    # Corner 1: West Runway (South) -> South Runway (East). X ~ -16.2 to -15.7, Y ~ -6.6 to -7.1
    # Corner 2 (Intersection): South Runway (East) -> Northbound turn. X ~ -6.0 to -2.3, Y ~ -7.1 to -4.0
    # Corner 3 (Bend / Hairpin): Northbound -> North transition -> Westbound runway. X ~ -2.3 to -8.0, Y ~ -4.0 to 7.1
    # Corner 4: North Runway (West) -> West Runway (South). X ~ -15.7 to -16.2, Y ~ 7.1 to 6.6

    print(f"\n[7] Corner Detailed Trajectory Breakdown (Corners 1, 2, 3, 4):")

    # Let's locate the 4 corners accurately using curvature peaks and spatial bounding boxes:
    # Corner 1:
    # X <= -15.0, Y <= -6.0, turning South to East (Yaw: -pi/2 -> 0.0)
    c1_mask = (wps[:, 0] <= -15.0) & (wps[:, 1] <= -6.0)
    c1_indices = np.where(c1_mask)[0]

    # Corner 2 (Intersection Turn):
    # X in [-6.5, -2.0], Y in [-7.5, -2.4], turning East to North (Yaw: 0.0 -> +pi/2)
    c2_mask = (
        (wps[:, 0] >= -6.8)
        & (wps[:, 0] <= -2.1)
        & (wps[:, 1] >= -7.3)
        & (wps[:, 1] <= -2.4)
    )
    c2_indices = np.where(c2_mask)[0]

    # Corner 3 (Sweeping Bend):
    # Hairpin curve continuing north and sweeping back west:
    # Y in [-2.4, 7.2], X >= -8.5, turning North to West (Yaw: +pi/2 -> +pi)
    c3_mask = (
        (wps[:, 0] >= -8.5)
        & (wps[:, 1] >= -2.4)
        & (wps[:, 1] <= 7.2)
        & (wps[:, 0] <= -2.0)
    )
    c3_indices = np.where(c3_mask)[0]

    # Corner 4:
    # X <= -15.0, Y >= 6.0, turning West to South (Yaw: pi -> -pi/2)
    c4_mask = (wps[:, 0] <= -15.0) & (wps[:, 1] >= 6.0)
    c4_indices = np.where(c4_mask)[0]

    def extract_corner_metrics(name, indices, expected_sign=1.0):
        # Among indices, find entry (where |kappa| starts exceeding 0.20),
        # apex (maximum |kappa|), and exit (where |kappa| falls below 0.20)
        k_sub = np.abs(kappa[indices])
        apex_sub_idx = np.argmax(k_sub)
        apex_idx = indices[apex_sub_idx]

        # Entry: first point in indices where |kappa| >= 0.15 (or first index)
        # Scan backward from apex
        entry_idx = indices[0]
        for idx in range(apex_idx, indices[0] - 1, -1):
            if np.abs(kappa[idx]) < 0.15:
                entry_idx = idx
                break

        # Exit: scan forward from apex
        exit_idx = indices[-1]
        for idx in range(apex_idx, indices[-1] + 1):
            if np.abs(kappa[idx]) < 0.15:
                exit_idx = idx
                break

        return {
            "name": name,
            "entry": entry_idx,
            "apex": apex_idx,
            "exit": exit_idx,
            "indices": indices,
            "max_kappa": kappa[apex_idx],
            "radius": 1.0 / abs(kappa[apex_idx])
            if abs(kappa[apex_idx]) > 1e-4
            else float("inf"),
        }

    corners = [
        ("Corner 1 (SW 90° Turn)", c1_indices),
        ("Corner 2 (Intersection 90° Turn)", c2_indices),
        ("Corner 3 (Sweeping Bend / Hairpin)", c3_indices),
        ("Corner 4 (NW 90° Turn)", c4_indices),
    ]

    corner_results = []
    for name, idxs in corners:
        res = extract_corner_metrics(name, idxs)
        corner_results.append(res)
        en, ap, ex = res["entry"], res["apex"], res["exit"]
        print(f"\n  * {res['name']}:")
        print(
            f"    - Entry : WP #{en:4d} | X={wps[en, 0]:8.4f}, Y={wps[en, 1]:8.4f}, Yaw={wps[en, 2]:7.4f} rad ({np.degrees(wps[en, 2]):6.1f}°) | kappa={kappa[en]:.4f} | dist_y={dist_y[en]:.3f}m, dist_w={dist_w[en]:.3f}m"
        )
        print(
            f"    - Apex  : WP #{ap:4d} | X={wps[ap, 0]:8.4f}, Y={wps[ap, 1]:8.4f}, Yaw={wps[ap, 2]:7.4f} rad ({np.degrees(wps[ap, 2]):6.1f}°) | kappa={kappa[ap]:.4f} (R={res['radius']:.3f}m) | dist_y={dist_y[ap]:.3f}m, dist_w={dist_w[ap]:.3f}m"
        )
        print(
            f"    - Exit  : WP #{ex:4d} | X={wps[ex, 0]:8.4f}, Y={wps[ex, 1]:8.4f}, Yaw={wps[ex, 2]:7.4f} rad ({np.degrees(wps[ex, 2]):6.1f}°) | kappa={kappa[ex]:.4f} | dist_y={dist_y[ex]:.3f}m, dist_w={dist_w[ex]:.3f}m"
        )

        # Sector bottleneck inside this corner
        c_dist_y = dist_y[idxs]
        c_dist_w = dist_w[idxs]
        c_min_y_i = idxs[np.argmin(c_dist_y)]
        c_min_w_i = idxs[np.argmin(c_dist_w)]
        print(
            f"    - Corner Tightest Yellow: {c_dist_y.min():.4f} m (WP #{c_min_y_i}) at ({wps[c_min_y_i, 0]:.3f}, {wps[c_min_y_i, 1]:.3f})"
        )
        print(
            f"    - Corner Tightest White : {c_dist_w.min():.4f} m (WP #{c_min_w_i}) at ({wps[c_min_w_i, 0]:.3f}, {wps[c_min_w_i, 1]:.3f})"
        )
        print(
            f"    - Corner Min Track Width: {(c_dist_y + c_dist_w).min():.4f} m (Mean: {(c_dist_y + c_dist_w).mean():.4f} m)"
        )

    # -------------------------------------------------------------
    # 6. Detailed Sector Breakdown & Pinch Point Forensics
    # -------------------------------------------------------------
    print(f"\n[8] Full-Circuit Sector Segmentation & Clearance Table:")
    # Segment circuit into 5 natural geometric phases:
    # Phase 1: West Runway (WP ~0 to Corner 1 entry)
    # Phase 2: Corner 1 (entry to exit)
    # Phase 3: South Runway (Corner 1 exit to Corner 2 entry)
    # Phase 4: East Loop / Hairpin (Corner 2 entry to Corner 3 exit)
    # Phase 5: North Runway & Corner 4

    print("-" * 88)
    print(
        f"{'Sector / Feature':<30} | {'WP Range':<11} | {'Min Y (m)':<9} | {'Min W (m)':<9} | {'Mean Width':<10} | {'Max Asym':<8}"
    )
    print("-" * 88)

    sectors_def = [
        (
            "West Runway (Sector 1/2)",
            np.where((wps[:, 0] <= -16.0) & (wps[:, 1] > -6.0) & (wps[:, 1] < 6.5))[0],
        ),
        ("Corner 1 Arc (Sector 3)", c1_indices),
        (
            "South Runway (Leg 2 Approach)",
            np.where((wps[:, 1] <= -6.9) & (wps[:, 0] > -15.5) & (wps[:, 0] < -6.8))[0],
        ),
        ("Corner 2 (Intersection)", c2_indices),
        ("Corner 3 (Sweeping Bend)", c3_indices),
        (
            "North Runway (Return Leg)",
            np.where((wps[:, 1] >= 6.9) & (wps[:, 0] > -15.5) & (wps[:, 0] < -8.0))[0],
        ),
        ("Corner 4 Arc", c4_indices),
    ]

    for s_name, s_idxs in sectors_def:
        if len(s_idxs) == 0:
            continue
        sy = dist_y[s_idxs]
        sw = dist_w[s_idxs]
        swidth = track_width[s_idxs]
        sasym = lane_asymmetry[s_idxs]
        wp_range_str = f"#{s_idxs[0]}-#{s_idxs[-1]}"
        print(
            f"{s_name:<30} | {wp_range_str:<11} | {sy.min():<9.4f} | {sw.min():<9.4f} | {swidth.mean():<10.4f} | {np.max(np.abs(sasym)):<8.4f}"
        )
    print("-" * 88)

    print("\n[PASS] Track Geometry Audit Completed Cleanly.")


if __name__ == "__main__":
    run_geometry_audit()
