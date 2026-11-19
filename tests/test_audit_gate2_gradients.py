#!/usr/bin/env python3
"""
Audit Gate 2: Asymmetric Actor-Critic Masking & Gradient Flow
Verification of source-level telemetry population, non-mutating clone, and autograd gradient flow
across all 5 masked Actor slots [5, 6, 8, 9, 11] and all 12 unmasked Critic slots.
"""

import os
import sys

import gymnasium as gym
import pytest
import torch

ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ARCPRO_LAB_DIR = os.path.join(ROOT_DIR, "arcproLab")
if ROOT_DIR not in sys.path:
    sys.path.insert(0, ROOT_DIR)
if ARCPRO_LAB_DIR not in sys.path:
    sys.path.insert(0, ARCPRO_LAB_DIR)

from agents.skrl_models import ARCProActor, ARCProCritic  # noqa: E402
from mdp.track_topology import ACTOR_MASKED_TELEMETRY_INDICES  # noqa: E402

pytestmark = pytest.mark.cpu


def test_source_code_inspection():
    obs_file = os.path.join(ARCPRO_LAB_DIR, "mdp", "observations.py")
    with open(obs_file, "r") as f:
        obs_code = f.read()

    assert 'obs[:, 8] = state["lat_err"]' in obs_code or 'obs[:, 8] = env.extras["lat_err"]' in obs_code
    assert 'obs[:, 9] = state["head_err"]' in obs_code or 'obs[:, 9] = env.extras["head_err"]' in obs_code
    assert 'obs[:, 10] = state["lookahead_kappa"]' in obs_code or 'obs[:, 10] = env.extras["lookahead_kappa"]' in obs_code
    assert 'obs[:, 11] = state["distance"]' in obs_code or 'obs[:, 11] = env.extras["distance"]' in obs_code

    models_file = os.path.join(ARCPRO_LAB_DIR, "agents", "skrl_models.py")
    with open(models_file, "r") as f:
        models_code = f.read()

    assert ".clone()" in models_code, "Missing .clone() call before setting telemetry masks!"
    assert "ACTOR_MASKED_TELEMETRY_INDICES" in models_code
    assert tuple(ACTOR_MASKED_TELEMETRY_INDICES) == (5, 6, 8, 9, 11)


def test_actor_critic_gradient_flow():
    device = "cpu"
    batch_size = 4
    num_visual = 25088
    num_telemetry = 12
    total_dim = num_visual + num_telemetry

    obs_space = gym.spaces.Box(-float("inf"), float("inf"), (total_dim,))
    act_space = gym.spaces.Box(-1.0, 1.0, (2,))

    torch.manual_seed(42)
    actor = ARCProActor(obs_space, act_space, device).to(device)
    critic = ARCProCritic(obs_space, act_space, device).to(device)

    # Critic Gradient Flow Check across all 12 telemetry slots
    x_critic = torch.randn(batch_size, total_dim, device=device, requires_grad=True)
    critic.zero_grad()
    values, _ = critic.compute({"states": x_critic})
    values.sum().backward()
    assert x_critic.grad is not None

    for slot_idx in range(12):
        g = x_critic.grad[:, num_visual + slot_idx].abs().sum().item()
        assert g > 0.0, f"Critic gradient on slot {slot_idx} is 0.0!"

    # Actor Gradient Flow Check
    x_actor = torch.randn(batch_size, total_dim, device=device, requires_grad=True)
    actor.zero_grad()
    action_mean, log_std, _ = actor.compute({"states": x_actor})
    (action_mean[:, 0] * 1.3 + action_mean[:, 1] * 0.7).sum().backward()
    assert x_actor.grad is not None

    for slot_idx in ACTOR_MASKED_TELEMETRY_INDICES:
        g = x_actor.grad[:, num_visual + slot_idx].abs().sum().item()
        assert g == 0.0, f"Actor gradient on masked slot {slot_idx} is {g} != 0.0!"

    for slot_idx in [0, 10]:
        g = x_actor.grad[:, num_visual + slot_idx].abs().sum().item()
        assert g > 0.0, f"Actor gradient on unmasked slot {slot_idx} is 0.0!"


def test_input_tensor_immutability():
    device = "cpu"
    batch_size = 4
    num_visual = 25088
    total_dim = num_visual + 12

    obs_space = gym.spaces.Box(-float("inf"), float("inf"), (total_dim,))
    act_space = gym.spaces.Box(-1.0, 1.0, (2,))
    actor = ARCProActor(obs_space, act_space, device).to(device)

    input_tensor = torch.zeros((batch_size, total_dim), device=device)
    for idx in ACTOR_MASKED_TELEMETRY_INDICES:
        input_tensor[:, num_visual + idx] = float(10 * (idx + 1))

    input_snapshot = input_tensor.clone()
    _ = actor.compute({"states": input_tensor})
    assert torch.equal(input_tensor, input_snapshot)


def test_perturbation_invariance():
    device = "cpu"
    batch_size = 4
    num_visual = 25088
    total_dim = num_visual + 12

    obs_space = gym.spaces.Box(-float("inf"), float("inf"), (total_dim,))
    act_space = gym.spaces.Box(-1.0, 1.0, (2,))
    actor = ARCProActor(obs_space, act_space, device).to(device)
    critic = ARCProCritic(obs_space, act_space, device).to(device)

    state_a = torch.randn(batch_size, total_dim, device=device)
    state_b = state_a.clone()
    state_b[:, num_visual + 5] += 50.0
    state_b[:, num_visual + 6] -= 50.0
    state_b[:, num_visual + 8] += 1000.0
    state_b[:, num_visual + 9] -= 500.0
    state_b[:, num_visual + 11] += 9999.0

    act_a, _, _ = actor.compute({"states": state_a})
    act_b, _, _ = actor.compute({"states": state_b})
    max_actor_diff = (act_a - act_b).abs().max().item()
    assert max_actor_diff == 0.0

    v_a, _ = critic.compute({"states": state_a})
    v_b, _ = critic.compute({"states": state_b})
    max_critic_diff = (v_a - v_b).abs().max().item()
    assert max_critic_diff > 0.1


if __name__ == "__main__":
    test_source_code_inspection()
    test_actor_critic_gradient_flow()
    test_input_tensor_immutability()
    test_perturbation_invariance()
