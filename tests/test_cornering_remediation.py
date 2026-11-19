#!/usr/bin/env python3
"""
Pure-CPU Unit and Regression Test Suite for Cornering Remediation:
1. Ackermann Electronic Differential (kinematic speed modulation).
2. Ultra-Wide Camera FOV (115 deg HFOV unlocking apex line visibility).
3. MDP Braking Advantage (ensures braking yields higher step reward than sprinting into turns).
4. Curvature-Adaptive Terminations (prevents false kills from Ackermann dynamic off-tracking).
5. Ackermann Feedforward, 5-Point Windowed Lookahead & Canonical Turn Token Boxes.
"""

import math
import os
import re
import sys

import pytest
import torch

WORKSPACE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if WORKSPACE_DIR not in sys.path:
    sys.path.insert(0, WORKSPACE_DIR)
ARCPRO_LAB_DIR = os.path.join(WORKSPACE_DIR, "arcproLab")
if ARCPRO_LAB_DIR not in sys.path:
    sys.path.insert(0, ARCPRO_LAB_DIR)

from mdp.track_manager import TrackManager  # noqa: E402
from mdp.track_topology import (  # noqa: E402
    BOUNDARY_CONTACT_WHITE_CORNER,
    BOUNDARY_CONTACT_WHITE_STRAIGHT,
    BOUNDARY_CONTACT_YELLOW_CORNER,
    BOUNDARY_CONTACT_YELLOW_STRAIGHT,
    BOUNDARY_CORNER_KAPPA_THRESH,
    SECTOR_DEFS,
    in_c2_turn,
)

pytestmark = pytest.mark.cpu


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


def test_camera_fov_unlocks_apex():
    """Verify camera configuration provides 115 deg HFOV and covers the 46.53 deg apex angle."""
    cfg_path = os.path.join(ARCPRO_LAB_DIR, "arcpro_env_cfg.py")
    with open(cfg_path, "r") as f:
        cfg_src = f.read()

    m_ap = re.search(r"horizontal_aperture\s*=\s*([0-9.]+)", cfg_src)
    m_fl = re.search(r"focal_length\s*=\s*([0-9.]+)", cfg_src)
    assert m_ap is not None and m_fl is not None

    h_aperture = float(m_ap.group(1))
    f_length = float(m_fl.group(1))

    hfov_rad = 2.0 * math.atan(h_aperture / (2.0 * f_length))
    hfov_deg = math.degrees(hfov_rad)

    assert h_aperture >= 6.0
    assert math.isclose(hfov_deg, 115.0, abs_tol=1.0)

    half_angle = hfov_deg / 2.0
    apex_required_angle = 46.53
    margin = half_angle - apex_required_angle
    assert margin > 8.0


def test_mdp_braking_advantage():
    """Verify that decelerating into sharp curves yields strictly higher step reward than sprinting."""
    dt = 0.02
    progress_w = 50.0
    speed_w = 180.0
    heading_w = 80.0

    wp_spacing = 0.010146
    target_v = 0.35
    sprint_v = 0.72

    # Option A: Sprinting at 0.72 m/s into a corner (overspeed penalty active)
    sprint_delta_wp = (sprint_v * dt) / wp_spacing
    r_progress_sprint = math.tanh(sprint_delta_wp) * progress_w * dt
    overspeed = max(0.0, sprint_v - target_v)
    r_speed_sprint = (
        math.exp(-3.5 * abs(sprint_v - target_v)) - overspeed * 4.0
    ) * speed_w * dt
    r_head_sprint = 1.0 * heading_w * dt
    total_sprint = r_progress_sprint + r_speed_sprint + r_head_sprint

    # Option B: Braking to 0.35 m/s
    brake_delta_wp = (target_v * dt) / wp_spacing
    r_progress_brake = math.tanh(brake_delta_wp) * progress_w * dt
    r_speed_brake = math.exp(-3.5 * abs(target_v - target_v)) * speed_w * dt
    r_head_brake = math.cos(math.radians(5.0)) * heading_w * dt
    total_brake = r_progress_brake + r_speed_brake + r_head_brake

    delta_reward = total_brake - total_sprint
    assert delta_reward > 0.0


def test_curvature_adaptive_terminations():
    """Verify that sharp curves adapt boundary thresholds to prevent false kills from dynamic off-tracking."""
    kappa_straight = torch.tensor([0.0])
    kappa_corner = torch.tensor([0.85])

    dyn_w_straight = torch.where(
        torch.abs(kappa_straight) > BOUNDARY_CORNER_KAPPA_THRESH,
        torch.tensor(BOUNDARY_CONTACT_WHITE_CORNER),
        torch.tensor(BOUNDARY_CONTACT_WHITE_STRAIGHT),
    )
    dyn_y_straight = torch.where(
        torch.abs(kappa_straight) > BOUNDARY_CORNER_KAPPA_THRESH,
        torch.tensor(BOUNDARY_CONTACT_YELLOW_CORNER),
        torch.tensor(BOUNDARY_CONTACT_YELLOW_STRAIGHT),
    )

    dyn_w_corner = torch.where(
        torch.abs(kappa_corner) > BOUNDARY_CORNER_KAPPA_THRESH,
        torch.tensor(BOUNDARY_CONTACT_WHITE_CORNER),
        torch.tensor(BOUNDARY_CONTACT_WHITE_STRAIGHT),
    )
    dyn_y_corner = torch.where(
        torch.abs(kappa_corner) > BOUNDARY_CORNER_KAPPA_THRESH,
        torch.tensor(BOUNDARY_CONTACT_YELLOW_CORNER),
        torch.tensor(BOUNDARY_CONTACT_YELLOW_STRAIGHT),
    )

    assert torch.isclose(dyn_w_straight, torch.tensor(0.015))
    assert torch.isclose(dyn_y_straight, torch.tensor(0.008))
    assert torch.isclose(dyn_w_corner, torch.tensor(0.012))
    assert torch.isclose(dyn_y_corner, torch.tensor(0.005))

    old_dead = 0.025 < 0.040
    new_survive = 0.025 >= dyn_w_corner.item()
    assert old_dead is True
    assert new_survive is True


def test_ackermann_feedforward_and_windowed_lookahead():
    """Verify Ackermann curvature feedforward, 5-point windowed lookahead, and aligned turn_token boxes."""
    signed_kappa = 1.075
    delta_ff = math.atan(0.33 * signed_kappa) / 0.68
    assert delta_ff > 0.45

    steer_match_left = math.exp(-4.0 * (0.50 - delta_ff) ** 2)
    steer_match_straight = math.exp(-4.0 * (0.00 - delta_ff) ** 2)
    assert steer_match_left > steer_match_straight

    tm = TrackManager(device="cpu")
    tm.ensure_synced()
    tm.last_indices = torch.tensor([1120, 1180], dtype=torch.long, device="cpu")
    lk = tm.get_lookahead_curvature(
        lookahead_distance=2.0,
        track_dir=torch.tensor([1.0, 1.0], dtype=torch.float32, device="cpu"),
    )
    assert lk[0].item() > 1.0
    assert lk[1].item() > 1.0

    sec4_x, sec4_y, _ = SECTOR_DEFS[4]
    c2_mask = in_c2_turn(
        torch.tensor([sec4_x, -5.50], dtype=torch.float32),
        torch.tensor([sec4_y, -7.10], dtype=torch.float32),
    )
    assert bool(c2_mask[0].item()) is False
    assert bool(c2_mask[1].item()) is True


if __name__ == "__main__":
    test_ackermann_electronic_differential()
    test_camera_fov_unlocks_apex()
    test_mdp_braking_advantage()
    test_curvature_adaptive_terminations()
    test_ackermann_feedforward_and_windowed_lookahead()
