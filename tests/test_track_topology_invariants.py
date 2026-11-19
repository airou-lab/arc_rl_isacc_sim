#!/usr/bin/env python3
"""
Pure-CPU Invariant Suite for Track Topology, Boundary Clearance, Single-Pass Step Cache,
and MDP Reward/Termination Consistency (`tests/test_track_topology_invariants.py`).
"""

import os
import re
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

import mdp.observations as mdp_obs  # noqa: E402
import mdp.rewards as mdp_rew  # noqa: E402
import mdp.terminations as mdp_term  # noqa: E402
import mdp.track_topology as topo  # noqa: E402
from mdp.road_manager import get_road_manager  # noqa: E402
from mdp.track_manager import get_track_manager  # noqa: E402
from mdp.track_topology import (  # noqa: E402
    BOUNDARY_WARN_MARGIN,
    SECTOR_DEFS,
    compute_step_road_state,
    in_any_corner,
    in_bend_turn,
    in_c2_turn,
    invalidate_step_cache,
)

pytestmark = pytest.mark.cpu


def test_track_topology_zero_isaac_omni_imports():
    """Verify track_topology.py imports with zero isaaclab/omni modules in sys.modules."""
    forbidden = [
        m for m in sys.modules if m.startswith("isaaclab") or m.startswith("omni")
    ]
    assert len(forbidden) == 0, f"Forbidden simulator modules loaded: {forbidden}"


def test_track_boundaries_1x_zero_violations_and_zero_holes():
    """Verify 0 yellow/white violations (< 0.170m) across all 5,000 WPs and 0 C2/Bend holes."""
    wp_path = os.path.join(ARCPRO_LAB_DIR, "mdp", "track_centerline.npy")
    bounds_path = os.path.join(ARCPRO_LAB_DIR, "mdp", "track_boundaries_1x.npz")

    wps = np.load(wp_path)
    bounds = np.load(bounds_path)
    yellow = bounds["yellow"][:, :2]
    white = bounds["white"][:, :2]

    assert wps.shape == (5000, 3)
    assert len(yellow) >= 9000
    assert len(white) >= 9000

    tree_y = cKDTree(yellow)
    tree_w = cKDTree(white)
    dy_all, _ = tree_y.query(wps[:, :2])
    dw_all, _ = tree_w.query(wps[:, :2])

    y_violations = int(np.sum(dy_all < BOUNDARY_WARN_MARGIN))
    w_violations = int(np.sum(dw_all < BOUNDARY_WARN_MARGIN))
    assert y_violations == 0, (
        f"Expected 0 yellow boundary violations (< {BOUNDARY_WARN_MARGIN}m), got {y_violations} (min={np.min(dy_all):.4f}m)"
    )
    assert w_violations == 0, (
        f"Expected 0 white boundary violations (< {BOUNDARY_WARN_MARGIN}m), got {w_violations} (min={np.min(dw_all):.4f}m)"
    )

    tx = torch.tensor(wps[:, 0], dtype=torch.float32)
    ty = torch.tensor(wps[:, 1], dtype=torch.float32)
    c2_mask = in_c2_turn(tx, ty).numpy()
    bend_mask = in_bend_turn(tx, ty).numpy()
    assert np.any(c2_mask) and np.any(bend_mask)

    assert float(np.max(dy_all[c2_mask])) < 0.25
    assert float(np.max(dw_all[c2_mask])) < 0.25
    assert float(np.max(dy_all[bend_mask])) < 0.25
    assert float(np.max(dw_all[bend_mask])) < 0.25


def test_sector_defs_spawns_safe_and_outside_corners():
    """Verify all 5 SECTOR_DEFS spawns (+ ±1cm lateral jitter) have dy >= 0.18, dw >= 0.18, and in_any_corner == False."""
    bounds_path = os.path.join(ARCPRO_LAB_DIR, "mdp", "track_boundaries_1x.npz")
    bounds = np.load(bounds_path)
    tree_y = cKDTree(bounds["yellow"][:, :2])
    tree_w = cKDTree(bounds["white"][:, :2])

    assert len(SECTOR_DEFS) == 5
    # Specifically verify Sector 3 is at -12.500 (not -15.200 inside C1 exit)
    assert math.isclose(SECTOR_DEFS[3][0], -12.500, abs_tol=1e-4)
    assert SECTOR_DEFS[3][0] != -15.200

    for sec_idx, (sx, sy, _yaw) in SECTOR_DEFS.items():
        for jx in (-0.01, 0.0, 0.01):
            for jy in (-0.01, 0.0, 0.01):
                px, py = float(sx) + jx, float(sy) + jy
                dy = float(tree_y.query([px, py])[0])
                dw = float(tree_w.query([px, py])[0])
                assert dy >= 0.18, (
                    f"Sector {sec_idx} jitter ({px:.3f}, {py:.3f}) dy={dy:.4f}m < 0.18m"
                )
                assert dw >= 0.18, (
                    f"Sector {sec_idx} jitter ({px:.3f}, {py:.3f}) dw={dw:.4f}m < 0.18m"
                )
                corner_flag = bool(
                    in_any_corner(
                        torch.tensor([px], dtype=torch.float32),
                        torch.tensor([py], dtype=torch.float32),
                    ).item()
                )
                assert corner_flag is False, (
                    f"Sector {sec_idx} spawn ({px:.3f}, {py:.3f}) is inside a corner!"
                )


import math  # noqa: E402


def test_zero_duplicated_corner_literals_outside_track_topology():
    """Scan consumer modules to guarantee zero duplicated hardcoded corner box literals exist outside track_topology.py."""
    target_files = [
        os.path.join(ARCPRO_LAB_DIR, "mdp", "road_manager.py"),
        os.path.join(ARCPRO_LAB_DIR, "mdp", "terminations.py"),
        os.path.join(ARCPRO_LAB_DIR, "mdp", "rewards.py"),
        os.path.join(ARCPRO_LAB_DIR, "mdp", "events.py"),
        os.path.join(ARCPRO_LAB_DIR, "scripts", "checkpoint_auditor.py"),
    ]
    banned_literals = ("-14.72", "-15.15", "-15.200", "-15.50", "-5.65")
    for fpath in target_files:
        with open(fpath, "r") as f:
            src = f.read()
        for lit in banned_literals:
            assert lit not in src, (
                f"Found banned hardcoded corner/spawn literal {lit!r} in {os.path.basename(fpath)}!"
            )


class _DummyRobotData:
    def __init__(self, num_envs: int, device: str = "cpu"):
        self.root_pos_w = torch.tensor(
            [[-16.197, 5.50, 0.025]] * num_envs, dtype=torch.float32, device=device
        )
        self.root_quat_w = torch.tensor(
            [[0.7071068, 0.0, 0.0, -0.7071068]] * num_envs,
            dtype=torch.float32,
            device=device,
        )
        self.root_lin_vel_b = torch.zeros(
            (num_envs, 3), dtype=torch.float32, device=device
        )
        self.root_lin_vel_b[:, 0] = 0.50
        self.root_ang_vel_b = torch.zeros(
            (num_envs, 3), dtype=torch.float32, device=device
        )
        self.joint_pos = torch.zeros((num_envs, 6), dtype=torch.float32, device=device)


class _DummyRobot:
    def __init__(self, num_envs: int, device: str = "cpu"):
        self.data = _DummyRobotData(num_envs, device)


class _DummyActionManager:
    def __init__(self, num_envs: int, device: str = "cpu"):
        self.action = torch.zeros((num_envs, 2), dtype=torch.float32, device=device)


class _DummyTerminationManager:
    def __init__(self, num_envs: int, device: str = "cpu"):
        self.terminated = torch.zeros(num_envs, dtype=torch.bool, device=device)


class _DummyScene(dict):
    def __init__(self, num_envs: int, device: str = "cpu"):
        super().__init__(
            robot=_DummyRobot(num_envs, device),
            sensors={},
        )
        self.env_origins = torch.zeros((num_envs, 3), dtype=torch.float32, device=device)


class _DummyEnv:
    def __init__(self, num_envs: int = 2, device: str = "cpu"):
        self.num_envs = num_envs
        self.device = device
        self.step_dt = 0.02
        self.common_step_counter = 1
        self.extras = {}
        self.scene = _DummyScene(num_envs, device)
        self.action_manager = _DummyActionManager(num_envs, device)
        self.termination_manager = _DummyTerminationManager(num_envs, device)
        self.reset_terminated = torch.zeros(num_envs, dtype=torch.bool, device=device)
        self.episode_length_buf = torch.full((num_envs,), 50, dtype=torch.long, device=device)


def test_single_cdist_per_step_and_cache_invalidation(monkeypatch):
    """Verify compute_step_road_state executes cdist only ONCE per step across terminations -> rewards -> observations."""
    env = _DummyEnv(num_envs=2, device="cpu")
    tm = get_track_manager(device="cpu")
    tm.ensure_synced()

    cdist_calls = {"tm": 0}
    orig_marker_dists = tm.compute_marker_distances

    def _wrapped_marker_dists(*args, **kwargs):
        cdist_calls["tm"] += 1
        return orig_marker_dists(*args, **kwargs)

    monkeypatch.setattr(tm, "compute_marker_distances", _wrapped_marker_dists)

    # 1. TerminationManager calls white_line_contact, height_termination, stagnation_termination
    _ = mdp_term.white_line_contact(env)
    _ = mdp_term.height_termination(env)
    _ = mdp_term.stagnation_termination(env)

    # 2. RewardManager calls all reward terms in the same step
    _ = mdp_rew.waypoint_progress_reward(env)
    _ = mdp_rew.heading_alignment_reward(env)
    _ = mdp_rew.lateral_error_reward(env)
    _ = mdp_rew.curvature_speed_reward(env)
    _ = mdp_rew.intersection_expert_reward(env)
    _ = mdp_rew.boundary_penalty(env)
    _ = mdp_rew.stationary_penalty(env)

    # 3. ObservationManager calls get_telemetry_vector in the same step
    obs = mdp_obs.get_telemetry_vector(env)
    assert obs.shape == (2, 12)

    assert cdist_calls["tm"] == 1, (
        f"Expected compute_marker_distances (3x cdist) to run exactly ONCE per step, got {cdist_calls['tm']} calls!"
    )

    # 4. Invalidate step cache on reset and verify next call recomputes
    invalidate_step_cache(env)
    _ = compute_step_road_state(env)
    assert cdist_calls["tm"] == 2


def test_reward_termination_math_and_red_light_stopping():
    """Verify termination_penalty (-50.0 at dt=0.02 -> -400.0 weighted), lateral_error_reward (-penalized_lat), and red-light stop behavior."""
    env = _DummyEnv(num_envs=2, device="cpu")
    env.reset_terminated = torch.tensor([True, False], dtype=torch.bool)

    term_pen = mdp_rew.termination_penalty(env)
    assert torch.isclose(term_pen[0], torch.tensor(-50.0))
    assert torch.isclose(term_pen[1], torch.tensor(0.0))
    # With RewardCfg.termination_penalty.weight = 400.0 and dt = 0.02, effective impulse is -50.0 * 400.0 * 0.02 = -400.0
    effective_impulse = term_pen[0].item() * 400.0 * env.step_dt
    assert math.isclose(effective_impulse, -400.0, rel_tol=1e-5)

    # Check RewardCfg weights in arcpro_env_cfg.py
    cfg_path = os.path.join(ARCPRO_LAB_DIR, "arcpro_env_cfg.py")
    with open(cfg_path, "r") as f:
        cfg_src = f.read()
    assert re.search(
        r"termination_penalty\s*=\s*RewTerm\([^)]*weight\s*=\s*400\.0",
        cfg_src,
        re.DOTALL,
    )
    assert re.search(
        r"lateral_error\s*=\s*RewTerm\([^)]*weight\s*=\s*225\.0",
        cfg_src,
        re.DOTALL,
    )

    # Verify lateral_error_reward returns negative penalty when |lat_err| > 0.04m on straight road (kappa=0)
    state = compute_step_road_state(env)
    state["kappa"] = torch.zeros(2, dtype=torch.float32)
    env.extras["kappa"] = state["kappa"]
    state["lat_err"] = torch.tensor([0.14, 0.02])
    env.extras["lat_err"] = state["lat_err"]
    lat_rew = mdp_rew.lateral_error_reward(env)
    assert torch.isclose(lat_rew[0], torch.tensor(-0.14), atol=1e-5)
    assert torch.isclose(lat_rew[1], torch.tensor(-0.02), atol=1e-5)

    # Verify red-light stopping inside stop bar (x=-7.0, y=-7.10, go_signal=0.0, v=0.0, drive=-1.0)
    env.scene["robot"].data.root_pos_w = torch.tensor(
        [[-7.00, -7.10, 0.025], [-7.00, -7.10, 0.025]], dtype=torch.float32
    )
    env.scene["robot"].data.root_lin_vel_b[:, 0] = 0.0
    env.action_manager.action[:, 0] = 0.0
    env.action_manager.action[:, 1] = -1.0  # stopped command
    invalidate_step_cache(env)
    state = compute_step_road_state(env)
    state["go_signal"] = torch.zeros(2, dtype=torch.float32)
    state["lat_err"] = torch.zeros(2, dtype=torch.float32)
    state["head_err"] = torch.zeros(2, dtype=torch.float32)
    state["kappa"] = torch.zeros(2, dtype=torch.float32)
    env.extras["go_signal"] = state["go_signal"]
    env.extras["lat_err"] = state["lat_err"]
    env.extras["head_err"] = state["head_err"]
    env.extras["kappa"] = state["kappa"]
    env.extras["cumulative_wp_index"] = torch.tensor([2.0, 2.0], dtype=torch.float32)

    spd_rew = mdp_rew.curvature_speed_reward(env)
    exp_rew = mdp_rew.intersection_expert_reward(env)
    stat_pen = mdp_rew.stationary_penalty(env)
    env.episode_length_buf = torch.tensor([200, 200], dtype=torch.long)
    stag_term = mdp_term.stagnation_termination(env)

    assert torch.allclose(spd_rew, torch.ones(2), atol=1e-4)
    assert torch.allclose(exp_rew, torch.ones(2), atol=1e-4)
    assert torch.allclose(stat_pen, torch.zeros(2), atol=1e-6)
    assert not bool(stag_term.any().item())
