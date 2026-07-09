"""Tests for SignalizedScheduler (scheduler-API adapter over the FSM).

Run from repo root:
  PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /home/arika/IsaacLab/isaaclab.sh -p -m pytest tests/test_signalized_scheduler.py -q
"""

import os
import sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "arcproLab", "mdp"))

import pytest

from signalized_scheduler import SignalizedScheduler


def make_sched(**kw):
    defaults = dict(
        green_s=(3.0, 2.0), yellow_s=1.0, max_green_s=6.0, step_dt=1.0,
        actuation_ext_s=2.0, actuation_window_s=1.5, randomize_phase=False,
    )
    defaults.update(kw)
    return SignalizedScheduler(**defaults)


def test_unknown_agent_fails_safe():
    s = make_sched()
    assert s.query_go_signal("ghost", "ix0") == 0.0
    assert s.phase("ghost") == 2


def test_register_returns_finite_horizon():
    s = make_sched()
    h = s.register_intent("a1", "ix0", "straight", approach_group=1)
    assert h == pytest.approx(4.0)  # group1 green starts after 3 (g0 green) + 1 (yellow)


def test_mutual_exclusion_over_cycles():
    s = make_sched()
    s.register_intent("ns", "ix0", "straight", approach_group=0)
    s.register_intent("ew", "ix0", "straight", approach_group=1)
    saw_ns = saw_ew = False
    for _ in range(30):
        go_ns = s.query_go_signal("ns", "ix0")
        go_ew = s.query_go_signal("ew", "ix0")
        assert go_ns + go_ew <= 1.0
        saw_ns |= go_ns == 1.0
        saw_ew |= go_ew == 1.0
        s.tick()
    assert saw_ns and saw_ew  # both approaches eventually served


def test_actuation_extends_green_to_cap():
    # With demand on the active green group, green lasts max_green (6)
    # instead of base (3); without demand, exactly 3.
    def green_duration(register: bool) -> int:
        s = make_sched()
        if register:
            s.register_intent("a", "ix0", "straight", approach_group=0)
        ticks = 0
        while s._spat(0)[2] == 0:  # while group 0 shows green
            s.tick()
            ticks += 1
            assert ticks < 50
        return ticks

    assert green_duration(register=False) == 3
    assert green_duration(register=True) == 6


def test_cross_street_not_starved():
    s = make_sched()
    s.register_intent("a", "ix0", "straight", approach_group=0)
    # run past the capped green + yellow; group 1 must eventually get green
    got_green = False
    for _ in range(12):
        s.tick()
        s.register_intent("b", "ix0", "straight", approach_group=1)
        if s.query_go_signal("b", "ix0") == 1.0:
            got_green = True
            break
    assert got_green


def test_clear_agent_removes_demand_and_permission():
    s = make_sched()
    s.register_intent("a", "ix0", "straight", approach_group=0)
    assert s.query_go_signal("a", "ix0") == 1.0  # group0 starts green
    s.clear_agent("a")
    assert s.query_go_signal("a", "ix0") == 0.0
    assert "a" not in s.active_intents


def test_yellow_queries_as_stop():
    s = make_sched()
    s.register_intent("a", "ix0", "straight", approach_group=0)
    for _ in range(6):
        s.tick()  # demand extends green to the 6-tick cap, then yellow
    assert s.phase("a") == 1
    assert s.query_go_signal("a", "ix0") == 0.0


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-q"]))
