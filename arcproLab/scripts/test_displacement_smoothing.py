#!/usr/bin/env python3
"""
Test Normal Midpoint Projection Centering.
"""

import os
import numpy as np
from scipy.spatial import cKDTree
from scipy.ndimage import gaussian_filter1d

def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    root_dir = os.path.abspath(os.path.join(script_dir, "..", ".."))
    
    orig_path = os.path.join(root_dir, "archive", "landmark_checkpoints", "track_centerline_pre_recalibration.npy")
    bounds_path = os.path.join(root_dir, "arcproLab", "mdp", "track_boundaries_1x.npz")
    
    orig = np.load(orig_path)
    bounds = np.load(bounds_path)
    yellow = bounds['yellow'][:, :2]
    white = bounds['white'][:, :2]
    
    tree_y = cKDTree(yellow)
    tree_w = cKDTree(white)
    
    n = len(orig)
    xy = orig[:, :2].copy()
    
    # Unit forward tangents from central differences
    next_xy = np.roll(xy, -1, axis=0)
    prev_xy = np.roll(xy, 1, axis=0)
    tangents = next_xy - prev_xy
    tangent_lens = np.linalg.norm(tangents, axis=1, keepdims=True)
    tangent_lens[tangent_lens == 0] = 1e-6
    t_unit = tangents / tangent_lens
    
    # Unit normal (90 deg CCW: [-t_y, t_x])
    n_unit = np.stack([-t_unit[:, 1], t_unit[:, 0]], axis=1)
    
    # Query nearest boundaries
    _, idx_y = tree_y.query(xy)
    _, idx_w = tree_w.query(xy)
    p_y = yellow[idx_y]
    p_w = white[idx_w]
    
    track_width = np.linalg.norm(p_y - p_w, axis=1)
    
    # Compute normal scalar offset delta to midpoint
    delta_raw = np.zeros(n)
    for i in range(n):
        # Exclude Corner 1 apex (WP 930 to 1020)
        if 930 <= i <= 1020:
            continue
            
        # Only in valid track corridors (width 35cm to 48cm)
        if 0.35 <= track_width[i] <= 0.48:
            mid = (p_y[i] + p_w[i]) / 2.0
            v = mid - xy[i]
            # Project onto normal vector
            delta = np.dot(v, n_unit[i])
            delta_raw[i] = np.clip(delta, -0.20, 0.20)
            
    print(f"[Analysis] Midpoint shift points: {np.sum(delta_raw != 0)} / {n}")
    print(f"[Analysis] Delta raw range: [{delta_raw.min()*100:.2f}, {delta_raw.max()*100:.2f}] cm")
    
    # Smooth delta field with periodic Gaussian filter
    delta_smooth = gaussian_filter1d(delta_raw, sigma=20.0, mode='wrap')
    
    # Apply normal shift: xy_centered = xy + delta_smooth * n_unit
    xy_centered = xy + delta_smooth[:, np.newaxis] * n_unit
    
    # Clearances
    d_y_new, _ = tree_y.query(xy_centered)
    d_w_new, _ = tree_w.query(xy_centered)
    
    apex_mask = np.zeros(n, dtype=bool)
    apex_mask[950:1001] = True
    
    print("\n--- Clearance Metrics ---")
    print(f"White clearance: min={d_w_new.min()*100:.2f} cm, mean={d_w_new.mean()*100:.2f} cm")
    print(f"Yellow clearance (apex): min={d_y_new[apex_mask].min()*100:.2f} cm")
    print(f"Yellow clearance (elsewhere): min={d_y_new[~apex_mask].min()*100:.2f} cm")
    print(f"WP 1871 white clearance (was 0.11cm): {d_w_new[1871]*100:.2f} cm")
    print(f"WP 1766 yellow clearance (was 2.8cm): {d_y_new[1766]*100:.2f} cm")
    print(f"WP 2490 yellow clearance (was 0.4cm): {d_y_new[2490]*100:.2f} cm")
    print(f"WP 4110 yellow clearance (was 0.8cm): {d_y_new[4110]*100:.2f} cm")
    print(f"WP 4230 white clearance (was 0.3cm):  {d_w_new[4230]*100:.2f} cm")
    
    # Tangent yaw from central differences
    next_c = np.roll(xy_centered, -1, axis=0)
    prev_c = np.roll(xy_centered, 1, axis=0)
    dx = next_c[:, 0] - prev_c[:, 0]
    dy = next_c[:, 1] - prev_c[:, 1]
    yaw = np.arctan2(-dy, -dx)
    
    def angle_diff(a, b):
        return (a - b + np.pi) % (2 * np.pi) - np.pi
    
    yaw_jumps = np.abs(angle_diff(np.roll(yaw, -1), yaw))
    print(f"\n--- Yaw Continuity Metrics ---")
    print(f"Max yaw jump (all 5000): {np.max(yaw_jumps):.4f} rad ({np.degrees(np.max(yaw_jumps)):.2f} deg)")
    print(f"Yaw jumps > 0.1 rad: {np.sum(yaw_jumps > 0.1)}")
    print(f"Yaw jumps > 0.05 rad: {np.sum(yaw_jumps > 0.05)}")
    
    hairpin_mask = np.zeros(n, dtype=bool)
    hairpin_mask[4930:4980] = True
    print(f"Max yaw jump (outside hairpin knot): {np.max(yaw_jumps[~hairpin_mask]):.4f} rad ({np.degrees(np.max(yaw_jumps[~hairpin_mask])):.2f} deg)")
    print(f"Yaw jumps > 0.05 rad (outside hairpin knot): {np.sum(yaw_jumps[~hairpin_mask] > 0.05)}")
    
    fwd_vec = next_c - xy_centered
    tangent_vec = np.stack([-np.cos(yaw), -np.sin(yaw)], axis=1)
    step_proj = np.sum(fwd_vec * tangent_vec, axis=1)
    print(f"Backwards steps: {np.sum(step_proj < 0)}")

if __name__ == "__main__":
    main()
