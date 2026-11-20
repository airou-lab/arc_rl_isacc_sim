#!/usr/bin/env python3
"""
Red-Team Adversarial Verification Suite for Milestone M006 Multi-Branch Intersection Gates:
  - Gate 1: Multi-Branch Splines & 3-Branch Tensor Bank (`TrackManager`) + 15,000-WP `cKDTree` Boundary Clearance Gate
  - Gate 2: Token-Gated Boundary Permeability & Intersection Envelopes (`track_topology.py`, `terminations.py`)
  - Gate 3: Vectorized `RoadManager` Turn-Token FSM, 4-Phase Lifecycle & Token-Bound MDP Rewards (`road_manager.py`, `rewards.py`, `events.py`)
"""

import math
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

import mdp.events as mdp_evt  # noqa: E402
import mdp.observations as mdp_obs  # noqa: E402
import mdp.rewards as mdp_rew  # noqa: E402
import mdp.terminations as mdp_term  # noqa: E402
from mdp.road_manager import RoadManager, get_road_manager  # noqa: E402
from mdp.track_manager import TrackManager, get_track_manager  # noqa: E402
from mdp.track_topology import (  # noqa: E402
    BOUNDARY_WARN_MARGIN,
    compute_step_road_state,
    in_branch_permeable_gate,
    in_junction_18_box,
    in_junction_55_box,
    in_multi_branch_intersection,
    in_off_cliff_east,
    in_wrong_lane_intersection,
    invalidate_step_cache,
)

pytestmark = pytest.mark.cpu


class _DummyRobotData:
    def __init__(self, num_envs: int, device: str = "cpu"):
        self.root_pos_w = torch.tensor(
            [[-5.50, -7.10, 0.025]] * num_envs, dtype=torch.float32, device=device
        )
        # Visual nose pointing East (+X, phys_yaw = 0.0 -> obs_yaw = pi -> track_dir = +1.0)
        self.root_quat_w = torch.tensor(
            [[1.0, 0.0, 0.0, 0.0]] * num_envs,
            dtype=torch.float32,
            device=device,
        )
        self.root_lin_vel_b = torch.zeros(
            (num_envs, 3), dtype=torch.float32, device=device
        )
        self.root_lin_vel_b[:, 0] = 0.35
        self.root_ang_vel_b = torch.zeros(
            (num_envs, 3), dtype=torch.float32, device=device
        )
        self.joint_pos = torch.zeros((num_envs, 6), dtype=torch.float32, device=device)
        self.joint_vel = torch.zeros((num_envs, 6), dtype=torch.float32, device=device)
        self.default_joint_pos = torch.zeros(
            (num_envs, 6), dtype=torch.float32, device=device
        )


class _DummyRobot:
    def __init__(self, num_envs: int, device: str = "cpu"):
        self.data = _DummyRobotData(num_envs, device)

    def write_root_pose_to_sim(
        self, pose: torch.Tensor, env_ids: torch.Tensor | None = None
    ) -> None:
        if env_ids is None:
            self.data.root_pos_w = pose[:, :3].clone()
            self.data.root_quat_w = pose[:, 3:7].clone()
        else:
            self.data.root_pos_w[env_ids] = pose[:, :3].clone()
            self.data.root_quat_w[env_ids] = pose[:, 3:7].clone()

    def write_root_velocity_to_sim(
        self, vel: torch.Tensor, env_ids: torch.Tensor | None = None
    ) -> None:
        if env_ids is None:
            self.data.root_lin_vel_b = vel[:, :3].clone()
        else:
            self.data.root_lin_vel_b[env_ids] = vel[:, :3].clone()

    def write_joint_state_to_sim(
        self,
        pos: torch.Tensor,
        vel: torch.Tensor,
        env_ids: torch.Tensor | None = None,
    ) -> None:
        if env_ids is None:
            self.data.joint_pos = pos.clone()
            self.data.joint_vel = vel.clone()
        else:
            self.data.joint_pos[env_ids] = pos.clone()
            self.data.joint_vel[env_ids] = vel.clone()


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
        self.num_envs = num_envs
        self.env_origins = torch.zeros((num_envs, 3), dtype=torch.float32, device=device)


class _DummyEnv:
    def __init__(self, num_envs: int = 3, device: str = "cpu"):
        self.num_envs = num_envs
        self.device = device
        self.step_dt = 0.02
        self.common_step_counter = 1
        self.extras = {}
        self.scene = _DummyScene(num_envs, device)
        self.action_manager = _DummyActionManager(num_envs, device)
        self.termination_manager = _DummyTerminationManager(num_envs, device)
        self.reset_terminated = torch.zeros(num_envs, dtype=torch.bool, device=device)
        self.episode_length_buf = torch.full(
            (num_envs,), 50, dtype=torch.long, device=device
        )


def test_gate1_multi_branch_spline_bank_shapes_and_48_env_vectorization():
    """Verify Gate 1 pre-materializes (3, 5000, 3) branch splines and handles N=48 mixed branches + 2D yaw cleanly."""
    tm = TrackManager(device="cpu")
    tm.ensure_synced()

    assert tm.branch_waypoints is not None
    assert tm.branch_curvatures is not None
    assert tm.branch_seg_bounds is not None
    assert tm.branch_waypoints.shape == (3, 5000, 3)
    assert tm.branch_curvatures.shape == (3, 5000)
    assert tm.branch_seg_bounds.shape == (3, 5000, 2)
    assert not torch.isnan(tm.branch_waypoints).any()
    assert not torch.isnan(tm.branch_curvatures).any()

    # Branch 0 must be bit-identical to canonical single-loop waypoints & curvature
    assert torch.equal(tm.branch_waypoints[0], tm.waypoints)
    assert torch.equal(tm.branch_curvatures[0], tm.curvature_tensor)

    # Test mixed 3-env batch at C2 intersection entry (-5.35, -7.10) heading East (obs_yaw = pi)
    pos = torch.tensor(
        [
            [-5.35, -7.10],  # Env 0: Left (-1.0 -> branch 0)
            [-5.35, -7.10],  # Env 1: Straight (0.0 -> branch 1)
            [-5.35, -7.10],  # Env 2: Right (+1.0 -> branch 2)
        ],
        dtype=torch.float32,
    )
    yaw = torch.tensor([math.pi, math.pi, math.pi], dtype=torch.float32)
    tokens = torch.tensor([-1.0, 0.0, 1.0], dtype=torch.float32)

    lat_err, head_err, kappa = tm.compute_errors(pos, yaw, turn_token=tokens)
    lk = tm.get_lookahead_curvature(
        lookahead_distance=1.0,
        track_dir=torch.ones(3, dtype=torch.float32),
        turn_token=tokens,
    )

    # Env 0 (Left): positive curvature ahead
    assert lk[0].item() > 0.25
    # Env 1 (Straight): zero lateral/heading error and zero curvature along Eastbound straight
    assert abs(lat_err[1].item()) < 1e-3
    assert abs(head_err[1].item()) < 1e-3
    assert abs(kappa[1].item()) < 1e-5
    assert abs(lk[1].item()) < 1e-5
    # Env 2 (Right): negative curvature (R=0.50m -> kappa=-2.0 m^-1)
    assert kappa[2].item() < -1.5 or lk[2].item() < -1.5

    # Verify N=48 mixed-branch vectorization & 2D (48, 1) yaw tensor broadcasting
    n_fleet = 48
    pos_48 = torch.tensor([[-5.35, -7.10, 0.0]] * n_fleet, dtype=torch.float32)
    yaw_48_2d = torch.full((n_fleet, 1), math.pi, dtype=torch.float32)
    branches_48 = torch.tensor([0, 1, 2] * (n_fleet // 3), dtype=torch.long)

    lat_48, head_48, kap_48 = tm.compute_errors(
        pos_48, yaw_48_2d, branch_ids=branches_48
    )
    lk_48 = tm.get_lookahead_curvature(
        lookahead_distance=1.0,
        track_dir=torch.ones(n_fleet, dtype=torch.float32),
        branch_ids=branches_48,
    )
    fwd_48 = tm.get_forward_waypoints(
        pos_48, yaw_48_2d, offsets=[20, 40, 60, 80, 100], branch_ids=branches_48
    )

    assert lat_48.shape == (n_fleet,)
    assert head_48.shape == (n_fleet,)
    assert kap_48.shape == (n_fleet,)
    assert lk_48.shape == (n_fleet,)
    assert fwd_48.shape == (n_fleet, 10)
    assert not torch.isnan(fwd_48).any()

    # Verify junction_55 (-16.205, -6.50) Straight (branch 1) and Right (branch 2)
    pos_j55 = torch.tensor(
        [
            [-16.205, -6.50],  # Straight Southbound
            [-17.200, -6.656],  # Exited Right Westbound on West Loop
        ],
        dtype=torch.float32,
    )
    yaw_j55 = torch.tensor([math.pi / 2.0, 0.0], dtype=torch.float32)
    b_j55 = torch.tensor([1, 2], dtype=torch.long)
    lat_j55, head_j55, kap_j55 = tm.compute_errors(pos_j55, yaw_j55, branch_ids=b_j55)
    assert abs(lat_j55[0].item()) < 1e-3
    assert abs(head_j55[0].item()) < 1e-3
    assert abs(kap_j55[0].item()) < 1e-5
    assert abs(lat_j55[1].item()) < 1e-2
    assert abs(head_j55[1].item()) < 1e-2

    # Verify junction_18 (-16.00, +7.10) Straight (branch 1) and (-15.752, +8.50) Right exit (branch 2)
    pos_j18 = torch.tensor(
        [
            [-16.000, 7.100],  # Straight Westbound across junction_18
            [-15.752, 8.500],  # Exited Right Northbound on Outer Grand Loop
        ],
        dtype=torch.float32,
    )
    yaw_j18 = torch.tensor([0.0, -math.pi / 2.0], dtype=torch.float32)
    b_j18 = torch.tensor([1, 2], dtype=torch.long)
    lat_j18, head_j18, kap_j18 = tm.compute_errors(pos_j18, yaw_j18, branch_ids=b_j18)
    assert abs(lat_j18[0].item()) < 1e-3
    assert abs(head_j18[0].item()) < 1e-3
    assert abs(kap_j18[0].item()) < 1e-5
    assert abs(lat_j18[1].item()) < 1e-2
    assert abs(head_j18[1].item()) < 1e-2


def test_gate1_full_15000_waypoint_boundary_clearance_gate():
    """Verify 0 yellow violations and 0 unmasked white violations (< 0.170m) across all 15,000 waypoints of Branches 0, 1, and 2."""
    tm = TrackManager(device="cpu")
    tm.ensure_synced()

    bounds_path = os.path.join(ARCPRO_LAB_DIR, "mdp", "track_boundaries_1x.npz")
    bounds = np.load(bounds_path)
    tree_y = cKDTree(bounds["yellow"][:, :2])
    tree_w = cKDTree(bounds["white"][:, :2])
    tree_g = cKDTree(bounds["gate"][:, :2])

    branch_tokens = {0: -1.0, 1: 0.0, 2: 1.0}
    for b_id, token_val in branch_tokens.items():
        wps_xy = tm.branch_waypoints[b_id, :, :2].numpy()
        dy, _ = tree_y.query(wps_xy)
        dw, _ = tree_w.query(wps_xy)
        dg, _ = tree_g.query(wps_xy)

        tx = torch.tensor(wps_xy[:, 0], dtype=torch.float32)
        ty = torch.tensor(wps_xy[:, 1], dtype=torch.float32)
        tg = torch.tensor(dg, dtype=torch.float32)

        perm_mask = in_branch_permeable_gate(
            tx,
            ty,
            tg,
            turn_token=token_val,
            multi_branch_active=(b_id > 0),
        ).numpy()

        y_violations = int(np.sum(dy < BOUNDARY_WARN_MARGIN))
        w_violations = int(np.sum((~perm_mask) & (dw < BOUNDARY_WARN_MARGIN)))

        assert y_violations == 0, (
            f"Branch {b_id} has {y_violations} yellow violations (< {BOUNDARY_WARN_MARGIN}m), min_dy={np.min(dy):.4f}m"
        )
        assert w_violations == 0, (
            f"Branch {b_id} has {w_violations} unmasked white violations (< {BOUNDARY_WARN_MARGIN}m), "
            f"min_dw_unmasked={np.min(dw[~perm_mask]):.4f}m"
        )


def test_gate2_token_gated_permeability_and_cliff_exemptions():
    """Verify Gate 2 masks outer white line crossings and eastern cliff ONLY when commanded Straight/Right."""
    # 1. Junction boxes
    assert bool(
        in_junction_55_box(torch.tensor([-15.971]), torch.tensor([-6.875])).item()
    )
    assert bool(
        in_junction_18_box(torch.tensor([-15.969]), torch.tensor([6.875])).item()
    )
    assert bool(
        in_multi_branch_intersection(
            torch.tensor([-15.971]), torch.tensor([-6.875])
        ).item()
    )

    # 2. East cliff at (X = -1.50, Y = -7.10): fatal on Left (-1.0) and Right (+1.0), exempt ONLY on Straight (0.0)
    x_east = torch.tensor([-1.50, -1.50, -1.50], dtype=torch.float32)
    y_east = torch.tensor([-7.10, -7.10, -7.10], dtype=torch.float32)
    tt = torch.tensor([-1.0, 0.0, 1.0], dtype=torch.float32)
    cliff_default = in_off_cliff_east(x_east, y_east)
    assert bool(cliff_default.all().item()) is True

    cliff_mb = in_off_cliff_east(
        x_east, y_east, turn_token=tt, multi_branch_active=True
    )
    assert bool(cliff_mb[0].item()) is True   # Left-turn agent killed at X=-1.50
    assert bool(cliff_mb[1].item()) is False  # Straight agent allowed to drive East!
    assert bool(cliff_mb[2].item()) is True   # Right-turn agent killed at X=-1.50

    # 3. White-line permeability inside intersection when crossing main-loop outer white boundary
    x_inter = torch.tensor([-3.50, -3.50, -3.50], dtype=torch.float32)
    y_inter = torch.tensor([-7.10, -7.10, -7.10], dtype=torch.float32)
    dg_far = torch.tensor([2.50, 2.50, 2.50], dtype=torch.float32)

    perm_default = in_branch_permeable_gate(x_inter, y_inter, dg_far)
    assert not bool(perm_default.any().item())

    perm_mb = in_branch_permeable_gate(
        x_inter, y_inter, dg_far, turn_token=tt, multi_branch_active=True
    )
    assert bool(perm_mb[0].item()) is False  # Left turn (-1.0): outer white line stays solid!
    assert bool(perm_mb[1].item()) is True   # Straight (0.0): outer white crossing is permeable!
    assert bool(perm_mb[2].item()) is True   # Right (+1.0): outer white crossing is permeable!


def test_gate3_vectorized_road_manager_and_token_bound_mdp_rewards():
    """Verify Gate 3 RoadManager FSM, curvature-symmetric apex slack, expert steering guidance, and wrong-branch termination."""
    env = _DummyEnv(num_envs=3, device="cpu")
    rm = get_road_manager(num_envs=3, num_agents=1, device="cpu")
    tm = get_track_manager(device="cpu")
    tm.ensure_synced()

    try:
        # 1. Default mode: deterministic -1.0 in corners, multi_branch_active == False
        rm.set_randomization(False)
        assert rm.is_multi_branch_active() is False

        # 2. Assign explicit mission tokens: Env 0 = Left (-1.0), Env 1 = Straight (0.0), Env 2 = Right (+1.0)
        rm.set_mission_tokens(torch.tensor([-1.0, 0.0, 1.0], dtype=torch.float32))
        assert rm.is_multi_branch_active() is True
        assert torch.equal(
            rm.get_active_branch_ids(), torch.tensor([0, 1, 2], dtype=torch.long)
        )

        # Place all 3 envs inside C2 intersection at (-5.35, -7.10) heading East (phys_yaw=0 -> obs_yaw=pi -> track_dir=+1)
        env.scene["robot"].data.root_pos_w = torch.tensor(
            [
                [-5.35, -7.10, 0.025],
                [-5.35, -7.10, 0.025],
                [-5.35, -7.10, 0.025],
            ],
            dtype=torch.float32,
        )
        env.scene["robot"].data.root_quat_w = torch.tensor(
            [[1.0, 0.0, 0.0, 0.0]] * 3, dtype=torch.float32
        )
        invalidate_step_cache(env)
        env.common_step_counter += 1

        terminated = mdp_term.white_line_contact(env)
        assert not bool(terminated.any().item()), (
            f"Expected 0 terminations at C2 entry across Left/Straight/Right, got {terminated}"
        )
        assert torch.equal(
            env.extras["track_dir"], torch.ones(3, dtype=torch.float32)
        ), f"Expected forward track_dir=+1.0, got {env.extras['track_dir']}"

        # Check telemetry observation slot 0 (turn_token)
        obs = mdp_obs.get_telemetry_vector(env)
        assert torch.allclose(
            obs[:, 0], torch.tensor([-1.0, 0.0, 1.0], dtype=torch.float32)
        )

        # 3. Verify intersection_expert_reward guides Left (+steer), Straight (0 steer), and Right (-steer)
        env.action_manager.action = torch.tensor(
            [[0.40, 0.0], [0.00, 0.0], [-0.60, 0.0]], dtype=torch.float32
        )
        r_expert_matched = mdp_rew.intersection_expert_reward(env)

        env.action_manager.action = torch.tensor(
            [[-0.50, 0.0], [0.60, 0.0], [0.50, 0.0]], dtype=torch.float32
        )
        r_expert_opposed = mdp_rew.intersection_expert_reward(env)

        assert bool((r_expert_matched > r_expert_opposed).all().item()), (
            f"Expert guidance must reward matching turn_token steering! matched={r_expert_matched}, opposed={r_expert_opposed}"
        )

        # 4. Verify curvature-sign-symmetric apex slack in lateral_error_reward
        env.extras["kappa"] = torch.tensor([2.0, -2.0, -2.0], dtype=torch.float32)
        env.extras["lat_err"] = torch.tensor([0.025, -0.025, 0.025], dtype=torch.float32)
        r_lat = mdp_rew.lateral_error_reward(env)
        assert abs(r_lat[0].item()) < 1e-6, f"Left inward apex should have 0 penalty, got {r_lat[0]}"
        assert abs(r_lat[1].item()) < 1e-6, f"Right inward apex should have 0 penalty, got {r_lat[1]}"
        assert math.isclose(r_lat[2].item(), -0.025, abs_tol=1e-5), (
            f"Right outward drift should be penalized -0.025, got {r_lat[2]}"
        )

        # 5. Advance each env onto its commanded exit branch:
        # Env 0 (Left): (-2.35, -3.00) heading North (phys_yaw=+pi/2 -> quat=[0.7071, 0, 0, +0.7071])
        # Env 1 (Straight): (-1.50, -7.10) heading East (phys_yaw=0 -> quat=[1, 0, 0, 0])
        # Env 2 (Right): (-5.00, -8.20) heading South (phys_yaw=-pi/2 -> quat=[0.7071, 0, 0, -0.7071])
        env.scene["robot"].data.root_pos_w = torch.tensor(
            [
                [-2.35, -3.00, 0.025],
                [-1.50, -7.10, 0.025],
                [-5.00, -8.20, 0.025],
            ],
            dtype=torch.float32,
        )
        env.scene["robot"].data.root_quat_w = torch.tensor(
            [
                [0.7071068, 0.0, 0.0, 0.7071068],
                [1.0, 0.0, 0.0, 0.0],
                [0.7071068, 0.0, 0.0, -0.7071068],
            ],
            dtype=torch.float32,
        )
        invalidate_step_cache(env)
        env.common_step_counter += 1

        term_exit = mdp_term.white_line_contact(env)
        assert bool(term_exit[1].item()) is False, "Straight Eastbound exit at X=-1.50 should NOT terminate!"
        assert bool(term_exit[2].item()) is False, "Right Southbound exit at (-5.00, -8.20) should NOT terminate!"
        assert torch.equal(
            env.extras["track_dir"], torch.ones(3, dtype=torch.float32)
        )

        # 6. Adversarial Wrong-Branch Cliff Termination:
        # Place ALL 3 agents (Left=-1.0, Straight=0.0, Right=+1.0) at (-1.50, -7.10) past the eastern cliff.
        # Left (Env 0) and Right (Env 2) MUST terminate; ONLY Straight (Env 1) is exempt!
        env.scene["robot"].data.root_pos_w = torch.tensor(
            [
                [-1.50, -7.10, 0.025],
                [-1.50, -7.10, 0.025],
                [-1.50, -7.10, 0.025],
            ],
            dtype=torch.float32,
        )
        env.scene["robot"].data.root_quat_w = torch.tensor(
            [[1.0, 0.0, 0.0, 0.0]] * 3, dtype=torch.float32
        )
        invalidate_step_cache(env)
        env.common_step_counter += 1

        term_wrong_branch = mdp_term.white_line_contact(env)
        assert torch.equal(
            term_wrong_branch, torch.tensor([True, False, True], dtype=torch.bool)
        ), f"Expected [True, False, True] at (-1.50, -7.10), got {term_wrong_branch}"

        # 7. Test uniform randomization sampling across {-1.0, 0.0, 1.0}
        rm_rand = RoadManager(num_envs=120, num_agents=1, device="cpu")
        rm_rand.set_randomization(True, allowed_tokens=[-1.0, 0.0, 1.0])
        unique_tokens = set(rm_rand.mission_tokens[:, 0].tolist())
        assert unique_tokens == {-1.0, 0.0, 1.0}, f"Expected all 3 tokens sampled, got {unique_tokens}"
    finally:
        rm.set_randomization(False)
        tm.reset_indices()


def test_gate3_four_phase_lifecycle_and_partial_reset_zero_progress_spike():
    """Verify approach -> intersection -> branch exit -> partial episode reset produces 0.0 spurious progress spike."""
    env = _DummyEnv(num_envs=3, device="cpu")
    rm = get_road_manager(num_envs=3, num_agents=1, device="cpu")
    tm = get_track_manager(device="cpu")
    tm.ensure_synced()

    try:
        # Phase 1: Start on Sector 4 approach (-6.55, -7.10, outside X >= -6.50 intersection box) with explicit missions [-1.0, 0.0, +1.0]
        rm.set_mission_tokens(
            torch.tensor([-1.0, 0.0, 1.0], dtype=torch.float32),
            latch_branch_immediately=False,
        )
        env.scene["robot"].data.root_pos_w = torch.tensor(
            [[-6.55, -7.10, 0.025]] * 3, dtype=torch.float32
        )
        env.scene["robot"].data.root_quat_w = torch.tensor(
            [[1.0, 0.0, 0.0, 0.0]] * 3, dtype=torch.float32
        )
        tm.reset_indices(new_pos=env.scene["robot"].data.root_pos_w[:, :2])
        invalidate_step_cache(env)
        env.common_step_counter += 1

        _ = mdp_term.white_line_contact(env)
        r_wp_0 = mdp_rew.waypoint_progress_reward(env)
        assert torch.allclose(r_wp_0, torch.zeros(3))

        # Phase 2: Enter intersection at (-6.45, -7.10) -> branches switch to [0, 1, 2]
        env.scene["robot"].data.root_pos_w = torch.tensor(
            [[-6.45, -7.10, 0.025]] * 3, dtype=torch.float32
        )
        invalidate_step_cache(env)
        env.common_step_counter += 1

        _ = mdp_term.white_line_contact(env)
        r_wp_enter = mdp_rew.waypoint_progress_reward(env)
        # Branch 1 & 2 re-anchored on switch -> zero spurious spike; Branch 0 advanced 10cm (~10 WPs)
        assert r_wp_enter[0].item() > 8.0
        assert r_wp_enter[1].item() == 0.0
        assert r_wp_enter[2].item() == 0.0

        # Phase 3: Advance 5cm East along each branch at (-6.40, -7.10) -> all 3 accumulate positive progress (~5 WPs)
        env.scene["robot"].data.root_pos_w = torch.tensor(
            [[-6.40, -7.10, 0.025]] * 3, dtype=torch.float32
        )
        invalidate_step_cache(env)
        env.common_step_counter += 1

        _ = mdp_term.white_line_contact(env)
        r_wp_step = mdp_rew.waypoint_progress_reward(env)
        assert bool((r_wp_step > 3.0).all().item()) and bool(
            (r_wp_step < 7.0).all().item()
        ), f"Expected ~5 WPs progress across all 3 branches, got {r_wp_step}"

        # Phase 4: Trigger episode reset via reset_robot_curriculum_spawn with randomization enabled
        rm.set_randomization(True, allowed_tokens=[0.0, 1.0])
        reset_ids = torch.tensor([0, 1, 2], dtype=torch.long)
        env.episode_length_buf[:] = 50
        mdp_evt.reset_robot_curriculum_spawn(env, reset_ids)

        # Simulate end-of-reset-step ObservationManager call (which populates the road state cache)
        _ = mdp_obs.get_telemetry_vector(env)

        # Simulate next physics step where robot is still at its spawn pose (0 movement)
        invalidate_step_cache(env)
        env.common_step_counter += 1
        _ = mdp_term.white_line_contact(env)
        r_wp_post_reset = mdp_rew.waypoint_progress_reward(env)

        assert torch.allclose(
            r_wp_post_reset, torch.zeros(3)
        ), f"Expected 0.0 progress on stationary step after reset, got {r_wp_post_reset}"
    finally:
        rm.set_randomization(False)
        tm.reset_indices()
