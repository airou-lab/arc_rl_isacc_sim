"""Integration test: RoadManager drives go_signals from the IntersectionManager.

CPU-only. Skips gate discovery (needs omni) by marking the manager
initialized; exercises exactly the code path the splice added.

Run from repo root:
  PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /home/arika/IsaacLab/isaaclab.sh -p -m pytest tests/test_roadmanager_v2i.py -q
"""

import os
import sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "arcproLab", "mdp"))

from types import SimpleNamespace

import torch
import pytest

from road_manager import RoadManager


def make_rm(num_envs=4):
    rm = RoadManager(num_envs=num_envs, num_agents=1, device="cpu")
    rm.initialized = True  # skip omni gate discovery on CPU
    return rm


def test_go_signal_toggles_over_a_cycle():
    rm = make_rm()
    env = SimpleNamespace()  # no reset_buf attribute -> getattr default path
    saw_go = torch.zeros(4, dtype=torch.bool)
    saw_stop = torch.zeros(4, dtype=torch.bool)
    # cycle = 8+2+8+2 = 20 s at step_dt 0.02 -> 1000 steps; run 1100
    for _ in range(1100):
        rm.update(env)
        _, go = rm.get_nav_commands()
        assert set(go.unique().tolist()) <= {0.0, 1.0}
        saw_go |= (go.view(-1) == 1.0)
        saw_stop |= (go.view(-1) == 0.0)
    assert saw_go.all(), "every env must see green within one cycle"
    assert saw_stop.all(), "every env must see stop within one cycle"


def test_time_to_change_is_normalized():
    rm = make_rm()
    env = SimpleNamespace()
    for _ in range(200):
        rm.update(env)
        ttc = rm.time_to_change
        assert torch.all(ttc >= 0.0) and torch.all(ttc <= 1.0)


def test_reset_rerandomizes_phase():
    rm = make_rm()
    env = SimpleNamespace(reset_buf=torch.zeros(4, dtype=torch.bool))
    rm.update(env)
    before = rm.intersection.t_rem.clone()
    env.reset_buf = torch.ones(4, dtype=torch.bool)
    rm.update(env)
    after = rm.intersection.t_rem
    # 4 envs re-randomized; all-equal to before is vanishingly unlikely
    assert not torch.allclose(before, after)


def test_legacy_fallback_when_disabled():
    rm = make_rm()
    rm.intersection = None
    env = SimpleNamespace()
    for _ in range(50):
        rm.update(env)
        _, go = rm.get_nav_commands()
        assert torch.all(go == 1.0)


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-q"]))
