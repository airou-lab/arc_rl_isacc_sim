"""Tests for IntersectionManager and V2IMessageBuffer.

Run from repo root:  python3 -m pytest tests/test_v2i.py -q
CPU-only, no Isaac required. step_dt=1.0 in FSM tests for easy tick counting.
"""

import os
import sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "arcproLab", "mdp"))

import torch
import pytest

from intersection_manager import IntersectionManager, GREEN, YELLOW
from v2i_buffer import V2IMessageBuffer


def make_fsm(**kw):
    defaults = dict(
        num_envs=2, green_s=(3.0, 2.0), yellow_s=1.0, max_green_s=6.0,
        step_dt=1.0, randomize_phase=False, device="cpu",
    )
    defaults.update(kw)
    return IntersectionManager(**defaults)


def test_fixed_time_cycle_transitions():
    m = make_fsm()
    assert torch.all(m.group == 0) and torch.all(m.state == GREEN)
    for _ in range(3):
        m.step()
    assert torch.all(m.state == YELLOW)
    assert torch.allclose(m.t_rem, torch.full((2,), 1.0))
    m.step()
    assert torch.all(m.group == 1) and torch.all(m.state == GREEN)
    assert torch.allclose(m.t_rem, torch.full((2,), 2.0))
    for _ in range(2):
        m.step()
    assert torch.all(m.state == YELLOW)
    m.step()
    assert torch.all(m.group == 0) and torch.all(m.state == GREEN)


def test_mutual_exclusion_across_full_cycles():
    m = make_fsm()
    ag = torch.tensor([[0, 1], [0, 1]])
    for _ in range(2 * 7 + 3):  # cycle = 3+1+2+1 = 7
        go, _, _ = m.get_spat(ag)
        assert torch.all(go.sum(dim=1) <= 1.0)
        assert torch.all((go == 1.0).any(dim=1) == (m.state == GREEN))
        m.step()


def test_time_to_change_green_red_yellow():
    m = make_fsm()
    ag = torch.tensor([[0, 1], [0, 1]])
    _, ttc, ph = m.get_spat(ag)
    assert torch.allclose(ttc, torch.tensor([[3.0, 4.0], [3.0, 4.0]]))
    assert torch.equal(ph, torch.tensor([[0, 2], [0, 2]]))
    for _ in range(3):
        m.step()
    _, ttc, ph = m.get_spat(ag)
    assert torch.allclose(ttc, torch.tensor([[4.0, 1.0], [4.0, 1.0]]))
    assert torch.equal(ph, torch.tensor([[1, 2], [1, 2]]))


def test_extend_green_caps_at_max_and_ignores_yellow():
    m = make_fsm()  # green0=3, max=6
    m.extend_green(torch.ones(2, dtype=torch.bool), 10.0)
    assert torch.allclose(m.t_rem, torch.full((2,), 6.0))
    for _ in range(6):
        m.step()
    assert torch.all(m.state == YELLOW)
    before = m.t_rem.clone()
    m.extend_green(torch.ones(2, dtype=torch.bool), 5.0)
    assert torch.allclose(m.t_rem, before)


def test_reset_randomizes_within_green():
    m = IntersectionManager(
        num_envs=64, green_s=(3.0, 2.0), yellow_s=1.0,
        step_dt=1.0, randomize_phase=True, seed=0,
    )
    assert torch.all(m.state == GREEN)
    g = m.green_s[m.group]
    assert torch.all(m.t_rem > 0.0) and torch.all(m.t_rem <= g)
    assert torch.allclose(m.t_rem + m.t_elapsed, g)


def test_spat_shapes_multiagent():
    m = make_fsm(num_envs=4)
    ag = torch.zeros(4, 3, dtype=torch.long)
    go, ttc, ph = m.get_spat(ag)
    assert go.shape == ttc.shape == ph.shape == (4, 3)


def _msg(v, n=1, d=1):
    return torch.full((n, d), float(v))


def test_zero_latency_is_transparent():
    b = V2IMessageBuffer(num_envs=1, msg_dim=1)
    for i in range(4):
        held, valid, age = b.step(_msg(i))
        assert held.item() == float(i) and valid.item() and age.item() == 0.0


def test_latency_delays_delivery():
    b = V2IMessageBuffer(num_envs=1, msg_dim=1, latency_steps=2)
    outs = [b.step(_msg(i)) for i in range(5)]
    assert not outs[0][1].item() and not outs[1][1].item()
    assert outs[2][0].item() == 0.0 and outs[2][1].item()
    assert outs[4][0].item() == 2.0


def test_full_dropout_never_delivers():
    b = V2IMessageBuffer(num_envs=4, msg_dim=2, dropout_p=1.0, seed=0)
    for i in range(10):
        held, valid, _ = b.step(torch.ones(4, 2))
    assert not valid.any()
    assert torch.all(held == 0.0)


def test_beacon_interval_holds_between_ticks():
    b = V2IMessageBuffer(num_envs=1, msg_dim=1, beacon_interval=3)
    outs = [b.step(_msg(i)) for i in range(7)]
    assert outs[0][0].item() == 0.0
    assert outs[1][0].item() == 0.0 and outs[2][0].item() == 0.0
    assert outs[3][0].item() == 3.0
    assert outs[4][0].item() == 3.0 and outs[5][0].item() == 3.0
    assert outs[6][0].item() == 6.0


def test_staleness_invalidates_old_messages():
    b = V2IMessageBuffer(num_envs=1, msg_dim=1, beacon_interval=4, stale_after=2)
    outs = [b.step(_msg(i)) for i in range(5)]
    assert outs[0][1].item()
    assert outs[2][1].item()
    assert not outs[3][1].item()
    assert outs[4][1].item()


def test_reset_clears_receiver_state():
    b = V2IMessageBuffer(num_envs=2, msg_dim=1)
    b.step(torch.ones(2, 1))
    b.reset(torch.tensor([True, False]))
    assert b.held[0].item() == 0.0 and not torch.isfinite(b.age[0])
    assert b.held[1].item() == 1.0 and b.age[1].item() == 0.0


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-q"]))
