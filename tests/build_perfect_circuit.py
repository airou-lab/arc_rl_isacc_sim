import numpy as np
from scipy.spatial import cKDTree
from scipy.interpolate import splprep, splev, interp1d

def generate_hairpin_circuit(num_pts=5000):
    data = np.load('arcproLab/mdp/track_boundaries_1x.npz')
    yellow = data['yellow'][:, :2]
    white = data['white'][:, :2]
    tree_y = cKDTree(yellow)
    tree_w = cKDTree(white)

    # 1. West Runway: (-16.197, 5.50) down to (-16.197, -5.50)
    # Direction: South (-Y)
    y_west = np.linspace(5.50, -5.50, 40)
    seg1 = np.column_stack([np.full_like(y_west, -16.197), y_west])

    # 2. Corner 1: Smooth 90-deg left turn from (-16.197, -5.50) to (-14.50, -7.10)
    # Direction: South -> East
    t1 = np.linspace(np.pi, 1.5 * np.pi, 30)
    c1_x = -14.50 + 1.697 * np.cos(t1)
    c1_y = -5.50 + 1.600 * np.sin(t1)
    seg2 = np.column_stack([c1_x, c1_y])

    # 3. South Runway: (-14.50, -7.10) to (-6.00, -7.10)
    # Direction: East (+X)
    x_south = np.linspace(-14.50, -6.00, 35)
    seg3 = np.column_stack([x_south, np.full_like(x_south, -7.10)])

    # 4. Hairpin Curve (Outside Lane): (-6.00, -7.10) around apex (-2.34, 0.0) to (-6.00, 6.82)
    # Extract exact outside-lane midpoints
    mask_hy = (yellow[:, 0] >= -6.0) & (yellow[:, 0] <= 0.0) & (yellow[:, 1] >= -7.5) & (yellow[:, 1] <= 7.5)
    mask_hw = (white[:, 0] >= -6.0) & (white[:, 0] <= 0.0) & (white[:, 1] >= -7.5) & (white[:, 1] <= 7.5)
    hy, hw = yellow[mask_hy], white[mask_hw]
    tree_hw = cKDTree(hw)

    # Sort yellow points by angle around (-5.0, 0.0)
    center = np.array([-5.0, 0.0])
    angles = np.arctan2(hy[:, 1] - center[1], hy[:, 0] - center[0])
    order = np.argsort(angles)
    hy_sorted = hy[order]

    # Sample ~60 points along the yellow curve between Y = -6.5 and Y = 5.5
    sample_idxs = np.linspace(0, len(hy_sorted)-1, 60, dtype=int)
    hp_mids = []
    for py in hy_sorted[sample_idxs]:
        # Outer white is further from center than py
        r_y = np.linalg.norm(py - center)
        dists, w_idxs = tree_hw.query(py, k=25)
        cand_w = hw[w_idxs]
        outer_cand = cand_w[np.linalg.norm(cand_w - center, axis=1) > r_y + 0.15]
        if len(outer_cand) > 0:
            d_out = np.linalg.norm(outer_cand - py, axis=1)
            best_w = outer_cand[np.argmin(d_out)]
            hp_mids.append((py + best_w) / 2.0)
    hp_mids = np.array(hp_mids)

    # 5. North Transition and Runway:
    # From hairpin exit (-6.00, 6.82) to (-8.00, 7.10) to (-14.50, 7.10)
    # Transition points:
    trans_pts = np.array([
        [-6.016, 6.816],
        [-6.445, 6.937],
        [-6.975, 7.038],
        [-7.489, 7.085],
        [-8.000, 7.100],
    ])
    x_north = np.linspace(-8.00, -14.50, 30)
    seg5_straight = np.column_stack([x_north, np.full_like(x_north, 7.10)])
    seg5 = np.vstack([trans_pts, seg5_straight])

    # 6. Corner 4: Smooth 90-deg left turn from (-14.50, 7.10) to (-16.197, 5.50)
    # Direction: West -> South
    t4 = np.linspace(0.5 * np.pi, np.pi, 30)
    c4_x = -14.50 + 1.697 * np.cos(t4)
    c4_y = 5.50 + 1.600 * np.sin(t4)
    seg6 = np.column_stack([c4_x, c4_y])

    # Combine anchor points
    anchors = np.vstack([seg1, seg2, seg3, hp_mids, seg5, seg6])

    # Filter out near duplicates (closer than 5cm)
    diffs = np.linalg.norm(np.diff(anchors, axis=0), axis=1)
    keep = np.insert(diffs > 0.05, 0, True)
    clean_anchors = anchors[keep]

    # Fit a smooth periodic B-spline
    tck, u = splprep([clean_anchors[:, 0], clean_anchors[:, 1]], s=0.005, per=True)

    # Dense uniform arc-length sampling
    u_dense = np.linspace(0, 1, 50000, endpoint=False)
    x_dense, y_dense = splev(u_dense, tck)
    dense_pts = np.column_stack([x_dense, y_dense])

    # Calculate cumulative distance for equidistant sampling
    step_dists = np.linalg.norm(np.diff(dense_pts, axis=0, append=dense_pts[:1]), axis=1)
    cum_dist = np.insert(np.cumsum(step_dists), 0, 0.0)[:-1]
    total_length = cum_dist[-1] + step_dists[-1]

    target_distances = np.linspace(0, total_length, num_pts, endpoint=False)
    fx = interp1d(cum_dist, x_dense, kind='cubic', fill_value='extrapolate')
    fy = interp1d(cum_dist, y_dense, kind='cubic', fill_value='extrapolate')

    x_fine = fx(target_distances)
    y_fine = fy(target_distances)
    pts_fine = np.column_stack([x_fine, y_fine])

    # Calculate tangent vector using central differences with periodic wrap
    dx = np.zeros(num_pts)
    dy = np.zeros(num_pts)
    for i in range(num_pts):
        ip = (i + 1) % num_pts
        im = (i - 1) % num_pts
        dx[i] = x_fine[ip] - x_fine[im]
        dy[i] = y_fine[ip] - y_fine[im]

    # Tangent yaw: direction of vehicle travel
    tangent_yaw = np.arctan2(dy, dx)

    # In ARCPro convention, robot chassis yaw has +pi offset:
    # wp_yaw = tangent_yaw + pi (so forward driving has head_err = 0)
    wp_yaw = (tangent_yaw + np.pi + np.pi) % (2 * np.pi) - np.pi

    # Verify against physical boundaries
    d_y, _ = tree_y.query(pts_fine)
    d_w, _ = tree_w.query(pts_fine)

    # Yaw step continuity
    dyaw = np.diff(wp_yaw, append=wp_yaw[0])
    dyaw = (dyaw + np.pi) % (2 * np.pi) - np.pi

    print(f"=== Perfect Hairpin Circuit: {num_pts} Waypoints ===")
    print(f"Track Total Length: {total_length:.3f} m")
    print(f"Waypoint Spacing:   {total_length / num_pts * 100:.2f} cm")
    print(f"Dist to Yellow:     min={d_y.min():.3f}m, max={d_y.max():.3f}m, mean={d_y.mean():.3f}m")
    print(f"Dist to White:      min={d_w.min():.3f}m, max={d_w.max():.3f}m, mean={d_w.mean():.3f}m")
    print(f"Max Yaw Step:       {np.max(np.abs(dyaw)):.5f} rad ({np.degrees(np.max(np.abs(dyaw))):.3f} deg)")
    print(f"Loop Closure Error: {np.linalg.norm(pts_fine[0] - pts_fine[-1]):.5f} m")

    wps = np.column_stack([x_fine, y_fine, wp_yaw])

    # Roll array so that WP 0 is precisely at the Start Line (-16.197, 5.50)
    start_target = np.array([-16.197, 5.50])
    dists_to_start = np.linalg.norm(wps[:, :2] - start_target, axis=1)
    start_idx = np.argmin(dists_to_start)
    wps = np.roll(wps, -start_idx, axis=0)

    # Re-verify rolled WP 0
    print(f"Rolled WP 0 to exact Start Line: {wps[0, :2]} (target {start_target}) | error: {np.linalg.norm(wps[0, :2] - start_target):.5f}m")

    return wps

if __name__ == '__main__':
    res = generate_hairpin_circuit(5000)
    out_path = 'arcproLab/mdp/track_centerline.npy'
    np.save(out_path, res)
    print(f"Successfully saved {len(res)} waypoints to {out_path}")
    print("Start waypoint:", res[0])
    print("WP 1250:       ", res[1250])
    print("WP 2500 (Apex):", res[2500])
    print("WP 3750:       ", res[3750])
    print("End waypoint:  ", res[-1])
