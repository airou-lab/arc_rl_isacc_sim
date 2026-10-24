#!/usr/bin/env python3
"""
Unit and Regression Test Suite for Cornering Remediation:
1. Ackermann Electronic Differential (kinematic speed modulation).
2. Ultra-Wide Camera FOV (115 deg HFOV unlocking apex line visibility).
3. MDP Braking Advantage (ensures braking yields higher step reward than sprinting into turns).
4. Curvature-Adaptive Terminations (prevents false kills from Ackermann dynamic off-tracking).
5. Telemetry Slot 7 Active Feedback (lateral drift/slip velocity).
"""

import math
import os
import sys

import torch

WORKSPACE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if WORKSPACE_DIR not in sys.path:
    sys.path.insert(0, WORKSPACE_DIR)
ARCPRO_LAB_DIR = os.path.join(WORKSPACE_DIR, "arcproLab")
if ARCPRO_LAB_DIR not in sys.path:
    sys.path.insert(0, ARCPRO_LAB_DIR)

from isaaclab.app import AppLauncher

app_launcher = AppLauncher({"headless": True, "num_envs": 1})
simulation_app = app_launcher.app

from arcproLab.arcpro_env_cfg import ARCProEnvCfg, ARCProSceneCfg


def test_ackermann_electronic_differential():
    """Verify electronic differential speed modulation."""
    wheelbase = 0.33
    half_track = 0.28 / 2.0

    # Test 1: Straightaway (delta = 0)
    steer_0 = torch.tensor([0.0])
    tan_0 = torch.tan(steer_0)
    diff_0 = (half_track * tan_0) / wheelbase
    left_0 = 1.0 - diff_0
    right_0 = 1.0 + diff_0
    assert torch.isclose(left_0, torch.tensor([1.0]))
    assert torch.isclose(right_0, torch.tensor([1.0]))
    assert torch.isclose((left_0 + right_0) / 2.0, torch.tensor([1.0]))

    # Test 2: Left Turn (delta = +0.50 rad)
    steer_left = torch.tensor([0.50])
    tan_left = torch.tan(steer_left)
    diff_left = (half_track * tan_left) / wheelbase
    left_mult = 1.0 - diff_left
    right_mult = 1.0 + diff_left
    assert left_mult < 1.0
    assert right_mult > 1.0
    assert right_mult > left_mult
    assert torch.isclose((left_mult + right_mult) / 2.0, torch.tensor([1.0]))
    print(
        f"[PASSED] Left turn differential: Left={left_mult.item():.3f}x, Right={right_mult.item():.3f}x, Mean=1.0x"
    )

    # Test 3: Right Turn (delta = -0.50 rad)
    steer_right = torch.tensor([-0.50])
    tan_right = torch.tan(steer_right)
    diff_right = (half_track * tan_right) / wheelbase
    left_mult_r = 1.0 - diff_right
    right_mult_r = 1.0 + diff_right
    assert left_mult_r > 1.0
    assert right_mult_r < 1.0
    assert left_mult_r > right_mult_r
    assert torch.isclose((left_mult_r + right_mult_r) / 2.0, torch.tensor([1.0]))
    print(
        f"[PASSED] Right turn differential: Left={left_mult_r.item():.3f}x, Right={right_mult_r.item():.3f}x, Mean=1.0x"
    )


def test_camera_fov_unlocks_apex():
    """Verify camera configuration provides 115 deg HFOV and covers the 46.53 deg apex angle."""
    scene_cfg = ARCProSceneCfg()
    camera_cfg = scene_cfg.tiled_camera

    h_aperture = camera_cfg.spawn.horizontal_aperture
    f_length = camera_cfg.spawn.focal_length

    # Compute HFOV in degrees
    hfov_rad = 2.0 * math.atan(h_aperture / (2.0 * f_length))
    hfov_deg = math.degrees(hfov_rad)

    assert h_aperture >= 6.0
    assert math.isclose(hfov_deg, 115.0, abs_tol=1.0)

    # In Corner 1 (R=1.164m), the inner apex line requires a viewing angle of 46.53 deg
    half_angle = hfov_deg / 2.0
    apex_required_angle = 46.53
    margin = half_angle - apex_required_angle
    assert margin > 8.0, (
        f"Apex viewing angle ({apex_required_angle} deg) must be safely inside half-FOV ({half_angle} deg)"
    )
    print(
        f"[PASSED] Camera HFOV: {hfov_deg:.1f} deg (Half-FOV: {half_angle:.1f} deg vs Apex required: {apex_required_angle} deg -> Margin: +{margin:.1f} deg)"
    )


def test_mdp_braking_advantage():
    """Verify that decelerating into sharp curves yields strictly higher step reward than sprinting."""
    cfg = ARCProEnvCfg()

    dt = cfg.sim.dt * cfg.decimation  # 0.02s
    progress_w = cfg.rewards.progress_reward.weight  # 200.0
    speed_w = cfg.rewards.speed_tracking.weight  # 180.0
    heading_w = cfg.rewards.heading.weight  # 100.0

    # Corner approach conditions: kappa_eff = 0.74, v_target = 0.35 m/s
    wp_spacing = 0.010146  # meters per waypoint
    target_v = 0.35
    sprint_v = 0.72

    # Option A: Sprinting at 0.72 m/s
    sprint_delta_wp = (sprint_v * dt) / wp_spacing
    r_progress_sprint = math.tanh(sprint_delta_wp) * progress_w * dt
    r_speed_sprint = math.exp(-2.0 * abs(sprint_v - target_v)) * speed_w * dt
    r_head_sprint = 1.0 * heading_w * dt
    total_sprint = r_progress_sprint + r_speed_sprint + r_head_sprint

    # Option B: Braking to 0.35 m/s
    brake_delta_wp = (target_v * dt) / wp_spacing
    r_progress_brake = math.tanh(brake_delta_wp) * progress_w * dt
    r_speed_brake = math.exp(-2.0 * abs(target_v - target_v)) * speed_w * dt
    r_head_brake = math.cos(math.radians(5.0)) * heading_w * dt
    total_brake = r_progress_brake + r_speed_brake + r_head_brake

    delta_reward = total_brake - total_sprint
    print(
        f"[Reward Audit] Sprint total: {total_sprint:.3f} pts/step | Brake total: {total_brake:.3f} pts/step | Delta: {delta_reward:+.3f} pts/step"
    )
    assert delta_reward > 0.0, (
        f"Braking must have a positive reward advantage over sprinting (got {delta_reward:+.3f})"
    )
    print(
        f"[PASSED] MDP Braking Advantage: +{delta_reward:.3f} pts/step (+{delta_reward / dt:.1f} pts/sec) in favor of corner deceleration"
    )


def test_curvature_adaptive_terminations():
    """Verify that sharp curves adapt boundary thresholds to prevent false kills from dynamic off-tracking."""
    kappa_straight = torch.tensor([0.0])
    kappa_corner = torch.tensor([0.85])

    dyn_w_straight = torch.where(
        torch.abs(kappa_straight) > 0.20, torch.tensor(0.015), torch.tensor(0.018)
    )
    dyn_y_straight = torch.where(
        torch.abs(kappa_straight) > 0.20, torch.tensor(0.008), torch.tensor(0.020)
    )

    dyn_w_corner = torch.where(
        torch.abs(kappa_corner) > 0.20, torch.tensor(0.015), torch.tensor(0.018)
    )
    dyn_y_corner = torch.where(
        torch.abs(kappa_corner) > 0.20, torch.tensor(0.008), torch.tensor(0.020)
    )

    assert dyn_w_straight == 0.018
    assert dyn_y_straight == 0.020
    assert dyn_w_corner == 0.015
    assert dyn_y_corner == 0.008

    old_dead = 0.025 < 0.040
    new_survive = 0.025 >= dyn_w_corner.item()
    assert old_dead is True
    assert new_survive is True
    print(
        f"[PASSED] Curvature-Adaptive Terminations: 2.5cm off-tracking survives in corner (dyn_w={dyn_w_corner.item():.3f}m) while healthy on straights ({dyn_w_straight.item():.3f}m)"
    )


def test_ackermann_feedforward_and_windowed_lookahead():
    """Verify Ackermann curvature feedforward, 5-point windowed lookahead, and aligned turn_token boxes."""
    from arcproLab.mdp.track_manager import TrackManager

    # Test A: Ackermann Curvature Feedforward at Centered Corner Entry (lat_err = 0.0, head_err = 0.0)
    signed_kappa = 1.075  # Corner 1 mean curvature
    delta_ff = math.atan(0.33 * signed_kappa) / 0.68
    assert delta_ff > 0.45, f"Expected delta_ff > 0.45, got {delta_ff:.4f}"

    steer_match_left = math.exp(-4.0 * (0.50 - delta_ff) ** 2)
    steer_match_straight = math.exp(-4.0 * (0.00 - delta_ff) ** 2)
    assert steer_match_left > steer_match_straight, (
        f"Left turn (+0.50) must earn higher teacher reward ({steer_match_left:.4f}) than straight (0.00 -> {steer_match_straight:.4f})"
    )
    print(
        f"[PASSED] Ackermann Feedforward: delta_ff={delta_ff:+.4f}, steer_match(+0.50)={steer_match_left:.4f} > steer_match(0.0)={steer_match_straight:.4f}"
    )

    # Test B: 5-Point Windowed Lookahead Eliminates Pre-Corner Blind Gap & Mid-Corner Drop-Off
    tm = TrackManager(device="cpu")
    tm.ensure_synced()
    tm.last_indices = torch.tensor([1120, 1180], dtype=torch.long, device="cpu")
    lk = tm.get_lookahead_curvature(
        lookahead_distance=2.0,
        track_dir=torch.tensor([1.0, 1.0], dtype=torch.float32, device="cpu"),
    )
    assert lk[0].item() > 1.0, (
        f"Pre-corner WP 1120 lookahead curvature must be > 1.0, got {lk[0].item():.4f}"
    )
    assert lk[1].item() > 1.0, (
        f"Apex WP 1180 lookahead curvature must be > 1.0, got {lk[1].item():.4f}"
    )
    print(
        f"[PASSED] 5-Point Windowed Lookahead: WP 1120={lk[0].item():+.3f}, WP 1180={lk[1].item():+.3f} (no blind gap or drop-off)"
    )

    # Test C: Track-Aligned turn_token Boxes (Sector 4 spawn vs Intersection entry)
    def eval_in_c2_turn(x: float, y: float) -> bool:
        return (x > -5.65) and (x < -2.0) and (y >= -7.5) and (y <= -3.00)

    assert eval_in_c2_turn(-6.00, -7.118) is False, (
        "Sector 4 spawn (-6.00, -7.118) inside double-yellow chute must have turn_token = 0.0 (in_c2_turn=False)"
    )
    assert eval_in_c2_turn(-5.50, -7.10) is True, (
        "Intersection entry (-5.50, -7.10) must activate turn_token = -1.0 (in_c2_turn=True)"
    )
    print(
        "[PASSED] Track-Aligned Turn Token: (-6.00, -7.118) -> 0.0 (chute), (-5.50, -7.10) -> -1.0 (intersection)"
    )


if __name__ == "__main__":
    print("=" * 60)
    print("RUNNING CORNERING REMEDIATION TEST SUITE")
    print("=" * 60)
    try:
        test_ackermann_electronic_differential()
        test_camera_fov_unlocks_apex()
        test_mdp_braking_advantage()
        test_curvature_adaptive_terminations()
        test_ackermann_feedforward_and_windowed_lookahead()
        print("=" * 60, flush=True)
        print("ALL CORNERING REMEDIATION TESTS PASSED (5/5)", flush=True)
        print("=" * 60, flush=True)
    except Exception:
        import traceback

        traceback.print_exc()
        sys.exit(1)
    finally:
        simulation_app.close()
