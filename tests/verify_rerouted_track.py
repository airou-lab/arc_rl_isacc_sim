import os
import sys
import numpy as np
from scipy.spatial import cKDTree


def verify():
    print("=======================================================")
    print("  Hostile Multi-Metric Track & Waypoint Verification   ")
    print("=======================================================")

    wp_file = "arcproLab/mdp/track_centerline.npy"
    bounds_file = "arcproLab/mdp/track_boundaries_1x.npz"

    # 1. Existence and Shape Gate
    assert os.path.exists(wp_file), f"FAIL: {wp_file} does not exist"
    assert os.path.exists(bounds_file), f"FAIL: {bounds_file} does not exist"

    wps = np.load(wp_file)
    assert wps.shape == (5000, 3), f"FAIL: Expected shape (5000, 3), got {wps.shape}"
    assert not np.isnan(wps).any(), "FAIL: NaNs found in waypoints"
    assert not np.isinf(wps).any(), "FAIL: Infs found in waypoints"
    print(f"[PASS] Waypoint shape verified: {wps.shape} | No NaNs or Infs")

    # 2. Physical Asphalt & Boundary Coverage Gate
    bounds = np.load(bounds_file)
    yellow = bounds["yellow"][:, :2]
    white = bounds["white"][:, :2]
    tree_y = cKDTree(yellow)
    tree_w = cKDTree(white)

    # In the racetrack, the intersection and transition zones without yellow paint are:
    # C1: X in [-16.5, -14.0], Y in [-7.5, -5.0]
    # C4: X in [-16.5, -14.0], Y in [5.0, 7.5]
    # NE and SE paint transition gaps: X in [-6.0, -3.5]
    in_c1 = (
        (wps[:, 0] >= -16.5)
        & (wps[:, 0] <= -14.0)
        & (wps[:, 1] >= -7.5)
        & (wps[:, 1] <= -5.0)
    )
    in_c4 = (
        (wps[:, 0] >= -16.5)
        & (wps[:, 0] <= -14.0)
        & (wps[:, 1] >= 5.0)
        & (wps[:, 1] <= 7.5)
    )
    in_trans = (
        (wps[:, 0] >= -6.0)
        & (wps[:, 0] <= -3.5)
        & ((wps[:, 1] >= 5.5) | (wps[:, 1] <= -5.5))
    )
    in_gap = in_c1 | in_c4 | in_trans

    # Check non-gap waypoints (where full continuous yellow lines exist)
    normal_wps = wps[~in_gap]
    d_y, _ = tree_y.query(normal_wps[:, :2])
    d_w, _ = tree_w.query(normal_wps[:, :2])

    max_dy = np.max(d_y)
    max_dw = np.max(d_w)
    mean_dy = np.mean(d_y)
    mean_dw = np.mean(d_w)

    print(f"[INFO] Fully marked WPs: {len(normal_wps)} / {len(wps)}")
    print(
        f"       Dist to Yellow: min={np.min(d_y):.3f}m, max={max_dy:.3f}m, mean={mean_dy:.3f}m"
    )
    print(
        f"       Dist to White:  min={np.min(d_w):.3f}m, max={max_dw:.3f}m, mean={mean_dw:.3f}m"
    )

    # Both lines should be roughly 0.17 - 0.25m away (lane width ~0.44m)
    assert mean_dy < 0.25, f"FAIL: Mean distance to yellow too large: {mean_dy:.3f}m"
    assert mean_dw < 0.25, f"FAIL: Mean distance to white too large: {mean_dw:.3f}m"
    assert max_dy < 0.35, f"FAIL: Waypoints too far from yellow: {max_dy:.3f}m"
    assert max_dw < 0.35, f"FAIL: Waypoints too far from white: {max_dw:.3f}m"
    assert np.min(d_y) >= 0.15, (
        f"FAIL: Waypoints too close to yellow: {np.min(d_y):.3f}m"
    )
    assert np.min(d_w) >= 0.15, (
        f"FAIL: Waypoints too close to white: {np.min(d_w):.3f}m"
    )
    print(
        "[PASS] 100% of fully marked waypoints are centered within drivable lane boundaries (min clearance >= 15cm)"
    )

    # Global boundary gate across all 5000 waypoints
    d_y_all, _ = tree_y.query(wps[:, :2])
    d_w_all, _ = tree_w.query(wps[:, :2])
    assert np.min(d_y_all) >= 0.15, (
        f"FAIL: Global minimum yellow distance too small: {np.min(d_y_all):.3f}m"
    )
    assert np.min(d_w_all) >= 0.15, (
        f"FAIL: Global minimum white distance too small: {np.min(d_w_all):.3f}m"
    )
    print(
        f"[PASS] Global boundary clearance gate: min yellow = {np.min(d_y_all):.3f}m, min white = {np.min(d_w_all):.3f}m (both >= 0.15m)"
    )

    # 3. Void Traversal Gate (Zero Points in the Void)
    # The void gap identified earlier was X in [-2.0, 3.0] with no road at Y in [-8, -5] or [5, 8]
    # In fact, East road only exists for X > 3.0. Zero waypoints should enter X > -1.8!
    max_x = np.max(wps[:, 0])
    assert max_x < -2.0, (
        f"FAIL: Waypoints enter void gap! Max X = {max_x:.3f}m (must be < -2.0m)"
    )
    print(
        f"[PASS] Zero points in void gap. Track max X = {max_x:.3f}m (safe boundary is < -2.0m)"
    )

    # 4. Smoothness & Yaw Step Continuity Gate
    yaw = wps[:, 2]
    dyaw = np.diff(yaw, append=yaw[0])
    dyaw = (dyaw + np.pi) % (2 * np.pi) - np.pi
    max_yaw_step = np.max(np.abs(dyaw))
    print(
        f"[INFO] Max yaw step across full 5000 WPs: {max_yaw_step:.5f} rad ({np.degrees(max_yaw_step):.3f} deg)"
    )
    assert max_yaw_step < 0.03, f"FAIL: Yaw step too large: {max_yaw_step:.4f} rad"
    print("[PASS] Yaw continuity gate passed (< 0.03 rad everywhere)")

    # 5. Loop Closure Gate
    closure_err = np.linalg.norm(wps[0, :2] - wps[-1, :2])
    print(f"[INFO] Loop closure error: {closure_err:.5f} m")
    assert closure_err < 0.03, (
        f"FAIL: Loop closure error too large: {closure_err:.4f} m"
    )
    print(f"[PASS] Loop closure gate passed (< 0.03m)")

    # 6. Curriculum Sectors Alignment Gate
    # Test all 5 certified sector locations from events.py:
    sectors = [
        ("Sector 1 (Start Line)", np.array([-16.197, 5.50]), -1.5708),
        ("Sector 2 (C1 Approach)", np.array([-16.197, -3.50]), -1.5708),
        ("Sector 3 (Leg 2 Post-C1)", np.array([-12.000, -7.10]), 0.0),
        ("Sector 4 (Leg 2 Approach)", np.array([-8.000, -7.10]), 0.0),
        ("Sector 5 (North Return)", np.array([-8.000, 7.10]), 3.14159),
    ]

    for name, pos, expected_heading in sectors:
        dists = np.linalg.norm(wps[:, :2] - pos, axis=1)
        closest_idx = np.argmin(dists)
        pos_err = dists[closest_idx]
        closest_wp = wps[closest_idx]

        robot_obs_yaw = (expected_heading + np.pi + np.pi) % (2 * np.pi) - np.pi
        head_err = (robot_obs_yaw - closest_wp[2] + np.pi) % (2 * np.pi) - np.pi

        print(
            f"[INFO] {name}: Pos Err={pos_err:.4f}m | Head Err={head_err:.4f} rad ({np.degrees(head_err):.1f}°)"
        )
        assert pos_err < 0.05, f"FAIL: {name} pos error {pos_err:.4f}m > 0.05m"
        assert abs(head_err) < 0.05, (
            f"FAIL: {name} head error {head_err:.4f} rad > 0.05 rad"
        )

    print(
        "[PASS] All 5 Curriculum Sectors verified within < 0.05m pos and < 0.05 rad heading error"
    )

    # 7. TrackManager Integration Gate
    sys.path.insert(0, "arcproLab")
    from mdp.track_manager import TrackManager
    import torch

    class MockEnv:
        def __init__(self):
            self.device = "cpu"
            self.num_envs = 1

    env = MockEnv()
    tm = TrackManager(device="cpu")
    tm.ensure_synced()
    assert tm.waypoints is not None and len(tm.waypoints) == 5000, (
        "FAIL: TrackManager failed to load 5000 waypoints"
    )
    assert tm.curvature_tensor is not None and len(tm.curvature_tensor) == 5000, (
        "FAIL: Curvature tensor not computed"
    )
    assert not torch.isnan(tm.curvature_tensor).any(), (
        "FAIL: Curvature tensor contains NaNs"
    )

    # Test error computation at Start Line
    test_pos = torch.tensor([[-16.197, 5.50]])
    test_yaw = torch.tensor([1.5708])  # Heading South in observations frame
    lat_err, head_err, kappa = tm.compute_errors(test_pos, test_yaw)
    print(
        f"[INFO] TrackManager Start Line Test: lat_err={lat_err.item():.4f}m, head_err={head_err.item():.4f} rad, kappa={kappa.item():.4f}"
    )
    assert abs(lat_err.item()) < 0.05, (
        f"FAIL: TrackManager lat_err {lat_err.item():.4f}m too high"
    )
    assert abs(head_err.item()) < 0.05, (
        f"FAIL: TrackManager head_err {head_err.item():.4f} rad too high"
    )
    print("[PASS] TrackManager integration gate verified cleanly")

    print("\n>>> ALL HOSTILE VERIFICATION GATES PASSED DETERMINISTICALLY! <<<")


if __name__ == "__main__":
    verify()
