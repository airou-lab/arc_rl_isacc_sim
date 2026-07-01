"""Tests for intersection_rewards. CPU-only, uses a fake env namespace.

Run from repo root:
  PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /home/arika/IsaacLab/isaaclab.sh -p -m pytest tests/test_intersection_rewards.py -q
"""

import os
import sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "arcproLab", "mdp"))

from types import SimpleNamespace

import torch
import pytest

import intersection_rewards as ir


def make_env(vel_fwd, go=None, dist_g=None):
    """vel_fwd: list of forward speeds (positive = forward).

    Robot convention: forward is -X in the body frame, so root_lin_vel_b[:,0]
    is the negation of forward speed.
    """
    n = len(vel_fwd)
    vel = torch.zeros(n, 3)
    vel[:, 0] = -torch.tensor(vel_fwd, dtype=torch.float32)
    extras = {}
    if go is not None:
        extras["go_signal"] = torch.tensor(go, dtype=torch.float32)
    if dist_g is not None:
        extras["dist_g"] = torch.tensor(dist_g, dtype=torch.float32)
    robot = SimpleNamespace(data=SimpleNamespace(root_lin_vel_b=vel))
    return SimpleNamespace(
        num_envs=n, device="cpu", extras=extras, scene={"robot": robot}
    )


def test_defaults_match_legacy_speed_reward():
    env = make_env([2.0, 0.0, -3.0])
    r = ir.gated_speed_reward(env)
    assert torch.allclose(r, torch.tensor([2.0, 0.0, -1.0]))  # clamp at -1


def test_defaults_match_legacy_stationary_penalty():
    env = make_env([0.05, 2.0])
    r = ir.gated_stationary_penalty(env)
    assert torch.allclose(r, torch.tensor([-1.0, 0.0]))


def test_defaults_hold_and_violation_are_zero():
    env = make_env([0.0, 5.0])
    assert torch.all(ir.hold_at_red_reward(env) == 0.0)
    assert torch.all(ir.red_light_violation(env) == 0.0)
    assert not ir.red_light_violation_termination(env).any()


def test_speed_reward_zeroed_only_in_red_zone():
    # envs: [red+near, red+far, green+near]
    env = make_env([2.0, 2.0, 2.0], go=[0.0, 0.0, 1.0], dist_g=[1.0, 9.0, 1.0])
    r = ir.gated_speed_reward(env, approach_radius=3.0)
    assert torch.allclose(r, torch.tensor([0.0, 2.0, 2.0]))


def test_stationary_penalty_suspended_at_red():
    # both stopped: [correctly held at red, idling on green]
    env = make_env([0.0, 0.0], go=[0.0, 1.0], dist_g=[1.0, 1.0])
    r = ir.gated_stationary_penalty(env)
    assert torch.allclose(r, torch.tensor([0.0, -1.0]))


def test_hold_reward_requires_red_near_and_stopped():
    # [held at red, moving at red, stopped on green, stopped red but far]
    env = make_env(
        [0.0, 1.0, 0.0, 0.0],
        go=[0.0, 0.0, 1.0, 0.0],
        dist_g=[1.0, 1.0, 1.0, 9.0],
    )
    r = ir.hold_at_red_reward(env)
    assert torch.allclose(r, torch.tensor([1.0, 0.0, 0.0, 0.0]))


def test_violation_fires_only_moving_through_line_on_red():
    # [running the light, stopped at line on red, through line on green, red but outside radius]
    env = make_env(
        [2.0, 0.0, 2.0, 2.0],
        go=[0.0, 0.0, 1.0, 0.0],
        dist_g=[0.2, 0.2, 0.2, 2.0],
    )
    r = ir.red_light_violation(env, violation_radius=0.5)
    assert torch.allclose(r, torch.tensor([-1.0, 0.0, 0.0, 0.0]))
    d = ir.red_light_violation_termination(env, violation_radius=0.5)
    assert d.tolist() == [True, False, False, False]


def test_violation_ignores_reverse_creep():
    # planar speed uses magnitude: rolling backward through the zone on red
    # still counts as motion in the zone
    env = make_env([-2.0], go=[0.0], dist_g=[0.2])
    r = ir.red_light_violation(env)
    assert r.item() == -1.0


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-q"]))
