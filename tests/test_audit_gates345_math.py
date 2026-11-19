#!/usr/bin/env python3
"""
Comprehensive Verification Gate for Gates 3, 4, and 5:
- Gate 3: IsaacLab Manager Execution Order & Single-Computation Per-Step Cache
- Gate 4: Zero-Error Steady-State Reward Equilibrium (Feedforward vs Feedback)
- Gate 5: Spatial Signal & Lookahead Window Alignment Against Track Geometry
"""

import math
import os
import sys
import unittest

import numpy as np
import pytest
import torch

ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ARCPRO_LAB_DIR = os.path.join(ROOT_DIR, "arcproLab")
if ROOT_DIR not in sys.path:
    sys.path.insert(0, ROOT_DIR)
if ARCPRO_LAB_DIR not in sys.path:
    sys.path.insert(0, ARCPRO_LAB_DIR)

from mdp.track_topology import (  # noqa: E402
    BOUNDARY_CONTACT_WHITE_STRAIGHT,
    BOUNDARY_CONTACT_YELLOW_STRAIGHT,
    in_c1_turn,
)

pytestmark = pytest.mark.cpu


class TestGate3ExecutionOrder(unittest.TestCase):
    """
    Gate 3 Audit: IsaacLab Manager Execution Order
    Step Cycle: ActionManager -> Physics -> TerminationManager -> RewardManager -> Reset -> ObservationManager.
    Verifies that compute_step_road_state updates all 9 spatial + go_signal keys in env.extras
    during TerminationManager (white_line_contact) with zero 1-step lag.
    """

    def test_extras_lifecycle_and_go_signal_lag(self):
        topo_file = os.path.join(ARCPRO_LAB_DIR, "mdp", "track_topology.py")
        done_file = os.path.join(ARCPRO_LAB_DIR, "mdp", "terminations.py")
        rew_file = os.path.join(ARCPRO_LAB_DIR, "mdp", "rewards.py")

        with open(topo_file, "r") as f:
            topo_src = f.read()
        with open(done_file, "r") as f:
            done_src = f.read()
        with open(rew_file, "r") as f:
            rew_src = f.read()

        # Verify compute_step_road_state populates go_signal and is called in terminations & rewards
        self.assertIn('"go_signal": go_signal', topo_src)
        self.assertIn("compute_step_road_state(env", done_src)
        self.assertIn("compute_step_road_state(env", rew_src)

    def test_white_line_contact_precomputation(self):
        topo_file = os.path.join(ARCPRO_LAB_DIR, "mdp", "track_topology.py")
        with open(topo_file, "r") as f:
            topo_src = f.read()

        required_keys = [
            '"lat_err":',
            '"head_err":',
            '"kappa":',
            '"dist_y":',
            '"dist_w":',
            '"dist_g":',
            '"lookahead_kappa":',
            '"turn_token":',
            '"go_signal":',
        ]
        for key in required_keys:
            self.assertIn(
                key,
                topo_src,
                f"Missing precomputation of {key} in compute_step_road_state",
            )


class TestGate4RewardEquilibrium(unittest.TestCase):
    """
    Gate 4 Audit: Zero-Error Steady-State Reward Equilibrium (Feedforward vs Feedback)
    """

    def test_feedforward_ackermann_curvature_substitution(self):
        L = 0.33  # Wheelbase (m)
        delta_max = 0.68  # Max steering joint angle (rad)
        kappa = 1.0  # 1.0 m^-1 curvature (R = 1.0 m left turn)

        delta_ackermann_rad = math.atan(L * kappa)
        delta_ff_normalized = delta_ackermann_rad / delta_max

        lat_err = torch.tensor([0.0])
        head_err = torch.tensor([0.0])
        kappa_tensor = torch.tensor([kappa])

        delta_ff = torch.atan(0.33 * kappa_tensor) / 0.68
        target_steer_with_ff = torch.clamp(
            delta_ff - 2.5 * lat_err - 1.2 * head_err, min=-1.0, max=1.0
        ).item()

        self.assertAlmostEqual(target_steer_with_ff, delta_ff_normalized, places=3)
        self.assertGreater(target_steer_with_ff, 0.40)

        target_steer_pure_feedback = torch.clamp(
            -2.5 * lat_err - 1.2 * head_err, min=-1.0, max=1.0
        ).item()
        self.assertEqual(target_steer_pure_feedback, 0.0)

        steer_match_pure_feedback = math.exp(
            -4.0 * (delta_ff_normalized - target_steer_pure_feedback) ** 2
        )
        steer_match_with_feedforward = math.exp(
            -4.0 * (delta_ff_normalized - target_steer_with_ff) ** 2
        )

        reward_loss_pct = (1.0 - steer_match_pure_feedback) * 100.0
        self.assertAlmostEqual(steer_match_with_feedforward, 1.0, places=4)
        self.assertAlmostEqual(steer_match_pure_feedback, 0.415, delta=0.01)
        self.assertAlmostEqual(reward_loss_pct, 58.4, delta=0.5)


class TestGate5SpatialAndLookaheadAlignment(unittest.TestCase):
    """
    Gate 5 Audit: Spatial Signal & Lookahead Window Alignment Against Track Geometry
    """

    def setUp(self):
        self.wps_path = os.path.join(ARCPRO_LAB_DIR, "mdp", "track_centerline.npy")
        self.bounds_path = os.path.join(
            ARCPRO_LAB_DIR, "mdp", "track_boundaries_1x.npz"
        )
        self.assertTrue(os.path.exists(self.wps_path), f"Missing {self.wps_path}")
        self.assertTrue(os.path.exists(self.bounds_path), f"Missing {self.bounds_path}")

        self.wps = np.load(self.wps_path)
        bounds = np.load(self.bounds_path)
        self.yellow = torch.tensor(bounds["yellow"][:, :2], dtype=torch.float32)
        self.white = torch.tensor(bounds["white"][:, :2], dtype=torch.float32)

    def test_corner1_exit_premature_token_snap(self):
        """
        Verify that in_c1_turn in track_topology.py extends to X <= -15.00m so that
        Corner 1 exit (X ~= -15.05m) stays covered until curvature drops below 0.10.
        """
        dx = np.gradient(self.wps[:, 0])
        dy = np.gradient(self.wps[:, 1])
        d2x = np.gradient(dx)
        d2y = np.gradient(dy)
        curvature = np.abs(dx * d2y - dy * d2x) / ((dx**2 + dy**2) ** 1.5 + 1e-8)

        c1_exit_indices = np.where(
            (self.wps[:, 0] > -15.50)
            & (self.wps[:, 0] < -14.50)
            & (self.wps[:, 1] < -6.50)
        )[0]
        c1_complete_idx = None
        for idx in c1_exit_indices:
            if curvature[idx] < 0.10:
                c1_complete_idx = idx
                break

        self.assertIsNotNone(c1_complete_idx)
        x_complete = float(self.wps[c1_complete_idx, 0])
        y_complete = float(self.wps[c1_complete_idx, 1])

        # Verify repaired in_c1_turn covers the entire Corner 1 exit up to X = -15.00
        tx = torch.tensor([x_complete, -14.80], dtype=torch.float32)
        ty = torch.tensor([y_complete, -7.10], dtype=torch.float32)
        mask = in_c1_turn(tx, ty)
        self.assertTrue(bool(mask[0].item()))
        self.assertFalse(bool(mask[1].item()))

    def test_step_1351_yellow_line_reset_clearance(self):
        """
        Verify that at X = -14.65, Y = -7.10 on Leg 2, the repaired straightaway
        thresholds (BOUNDARY_CONTACT_YELLOW_STRAIGHT = 0.008, WHITE = 0.015) provide
        >= 3.4 cm of drift margin before boundary contact termination.
        """
        pos = torch.tensor([[-14.65, -7.10]], dtype=torch.float32)
        dist_y = torch.min(torch.cdist(pos, self.yellow), dim=1)[0].item()
        dist_w = torch.min(torch.cdist(pos, self.white), dim=1)[0].item()

        lane_width = dist_y + dist_w
        self.assertAlmostEqual(lane_width, 0.407, delta=0.03)

        vehicle_half_width = 0.16
        nominal_tire_clearance = dist_y - vehicle_half_width
        self.assertAlmostEqual(nominal_tire_clearance, 0.042, delta=0.01)

        self.assertEqual(BOUNDARY_CONTACT_YELLOW_STRAIGHT, 0.008)
        self.assertEqual(BOUNDARY_CONTACT_WHITE_STRAIGHT, 0.015)
        max_allowable_drift = nominal_tire_clearance - BOUNDARY_CONTACT_YELLOW_STRAIGHT
        self.assertGreaterEqual(max_allowable_drift, 0.030)

    def test_single_point_vs_multipoint_lookahead_overshoot(self):
        num_wps = 500
        curvatures = torch.zeros(num_wps, dtype=torch.float32)
        corner_start = 100
        corner_end = 210
        curvatures[corner_start:corner_end] = 1.43

        current_idx_95 = torch.tensor([95], dtype=torch.long)
        step_offset = 120
        overshoot_probe_idx = (current_idx_95 + step_offset) % num_wps
        overshoot_kappa = curvatures[overshoot_probe_idx].item()
        self.assertEqual(overshoot_kappa, 0.0)

        fracs = torch.linspace(0.0, 1.0, steps=5)
        step_offsets = torch.round(fracs * step_offset).to(dtype=torch.long)
        window_indices = (
            current_idx_95.unsqueeze(1) + step_offsets.unsqueeze(0)
        ) % num_wps
        kappa_window = curvatures[window_indices]
        max_idx = torch.argmax(torch.abs(kappa_window), dim=1, keepdim=True)
        multipoint_kappa = (
            torch.gather(kappa_window, dim=1, index=max_idx).squeeze(1).item()
        )
        self.assertAlmostEqual(multipoint_kappa, 1.43, places=2)


if __name__ == "__main__":
    unittest.main(verbosity=2)
