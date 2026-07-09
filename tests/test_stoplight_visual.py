"""Tests for StoplightVisual sync logic (CPU, mocked lamps — no pxr needed).

USD construction itself is validated in the GUI verify session; these tests
cover the parts that can break silently: env-path parsing, phase-to-lamp
mapping, and the touch-USD-only-on-transition guarantee.

Run from repo root:
  PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /home/arika/IsaacLab/isaaclab.sh -p -m pytest tests/test_stoplight_visual.py -q
"""

import os
import sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "arcproLab", "mdp"))

import torch
import pytest

from stoplight_visual import StoplightVisual


class MockLamp:
    def __init__(self):
        self.visible = None
        self.calls = 0

    def MakeVisible(self):
        self.visible = True
        self.calls += 1

    def MakeInvisible(self):
        self.visible = False
        self.calls += 1


def make_sv(num_envs=2):
    sv = StoplightVisual()
    for e in range(num_envs):
        sv._lamps_by_env[e] = {n: MockLamp() for n in StoplightVisual.LAMP_ORDER}
        sv._last_phase[e] = None
    return sv


def test_env_path_parsing():
    f = StoplightVisual._env_index
    assert f("/World/envs/env_0/Track/laneGate_3") == 0
    assert f("/World/envs/env_17/Track/laneGate_0") == 17
    assert f("/World/Track/laneGate") == 0  # no env namespace -> 0
    g = StoplightVisual._env_root
    assert g("/World/envs/env_17/Track/laneGate_0") == "/World/envs/env_17"
    assert g("/World/Track/laneGate") == "/World"


def test_sync_sets_exactly_one_lamp_visible():
    sv = make_sv(num_envs=1)
    sv.sync(torch.tensor([0]))  # green
    lamps = sv._lamps_by_env[0]
    assert lamps["green"].visible is True
    assert lamps["amber"].visible is False and lamps["red"].visible is False
    sv.sync(torch.tensor([2]))  # red
    assert lamps["red"].visible is True
    assert lamps["green"].visible is False and lamps["amber"].visible is False


def test_sync_touches_usd_only_on_transition():
    sv = make_sv(num_envs=1)
    sv.sync(torch.tensor([0]))
    calls_after_first = sum(l.calls for l in sv._lamps_by_env[0].values())
    for _ in range(100):
        sv.sync(torch.tensor([0]))  # same phase, no writes
    assert sum(l.calls for l in sv._lamps_by_env[0].values()) == calls_after_first
    sv.sync(torch.tensor([1]))  # transition -> writes again
    assert sum(l.calls for l in sv._lamps_by_env[0].values()) == calls_after_first + 3


def test_sync_is_per_env_independent():
    sv = make_sv(num_envs=2)
    sv.sync(torch.tensor([0, 2]))  # env0 green, env1 red
    assert sv._lamps_by_env[0]["green"].visible is True
    assert sv._lamps_by_env[1]["red"].visible is True
    assert sv._lamps_by_env[1]["green"].visible is False


def test_unknown_phase_defaults_to_red():
    sv = make_sv(num_envs=1)
    sv.sync(torch.tensor([7]))  # out-of-protocol value -> fail safe
    assert sv._lamps_by_env[0]["red"].visible is True


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-q"]))
