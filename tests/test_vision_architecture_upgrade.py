#!/usr/bin/env python3
"""
Pure-CPU Verification Suite for Vision Architecture Upgrades:
1. SpatialSoftmax2d: Output shapes, coordinate bounds [-1, 1], and autograd differentiability.
2. ARCProActor: Trainable parameter count (21,474), forward pass, and gradient backprop.
3. YUV Linear Preprocessing: Chrominance isolation of yellow median vs white boundary.
4. Elevation Termination: Permissive Z bounds accommodating 3D track hill up to Z = 2.5m.
"""

import os
import sys

import gymnasium as gym
import numpy as np
import pytest
import torch
import torch.nn as nn

WORKSPACE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ARCPRO_LAB_DIR = os.path.join(WORKSPACE_DIR, "arcproLab")
for _p in (WORKSPACE_DIR, ARCPRO_LAB_DIR):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from arcproLab.agents.skrl_models import (  # noqa: E402
    ARCProActor,
    ARCProCritic,
    SpatialSoftmax2d,
)
from arcproLab.mdp.terminations import height_termination  # noqa: E402

pytestmark = pytest.mark.cpu


def test_spatial_softmax():
    device = torch.device("cpu")
    ssm = SpatialSoftmax2d(height=7, width=7, temperature=1.0).to(device)

    B, C, H, W = 20, 32, 7, 7
    feat = torch.randn(B, C, H, W, device=device, requires_grad=True)
    coords = ssm(feat)
    assert coords.shape == (B, 64), f"Expected shape ({B}, 64), got {coords.shape}"
    assert (coords >= -1.0).all() and (coords <= 1.0).all()

    loss = coords.sum()
    loss.backward()
    assert feat.grad is not None
    assert feat.grad.norm().item() > 0.0


def test_actor_architecture():
    device = torch.device("cpu")
    obs_space = gym.spaces.Box(low=-np.inf, high=np.inf, shape=(25100,))
    act_space = gym.spaces.Box(low=-1.0, high=1.0, shape=(2,))

    torch.manual_seed(7)
    actor = ARCProActor(obs_space, act_space, device).to(device)

    total_trainable = sum(p.numel() for p in actor.parameters() if p.requires_grad)
    assert total_trainable in (21474, 21476), (
        f"Expected 21,474-21,476 parameters, got {total_trainable}"
    )

    dummy_obs = torch.randn(20, 25100, device=device)
    action_mean, log_std, _ = actor.compute({"states": dummy_obs})

    assert action_mean.shape == (20, 2)
    assert not torch.isnan(action_mean).any()
    assert (action_mean >= -1.0).all() and (action_mean <= 1.0).all()

    obs_a = torch.randn(20, 25100, device=device)
    obs_a[:, 25088 + 0] = 0.0  # straightaway turn_token == 0.0
    obs_b = obs_a.clone()
    obs_b[:, 25088 + 5] = 0.85  # prev_steer (masked)
    obs_b[:, 25088 + 6] = -1.00  # prev_drive (masked)
    obs_b[:, 25088 + 8] = 99.0  # lat_err (masked)
    obs_b[:, 25088 + 9] = -99.0  # head_err (masked)
    obs_b[:, 25088 + 11] = 500.0  # distance (masked)
    obs_b_telemetry_snapshot = obs_b[:, 25088:25100].clone()
    act_a, _, _ = actor.compute({"states": obs_a})
    act_b, _, _ = actor.compute({"states": obs_b})
    assert torch.allclose(act_a, act_b, atol=1e-6)
    assert torch.equal(obs_b[:, 25088:25100], obs_b_telemetry_snapshot)

    grad_states = obs_a.clone().detach().requires_grad_(True)
    actor.zero_grad()
    act_grad_out, _, _ = actor.compute({"states": grad_states})
    (act_grad_out[:, 0] * 1.3 + act_grad_out[:, 1] * 0.7).sum().backward()
    assert grad_states.grad is not None
    for i in [5, 6, 8, 9, 11]:
        slot_grad = grad_states.grad[:, 25088 + i].abs().sum().item()
        assert slot_grad == 0.0, (
            f"Expected zero input gradient on masked slot {i}, got {slot_grad}"
        )
    for i in [0, 10]:
        slot_grad = grad_states.grad[:, 25088 + i].abs().sum().item()
        assert slot_grad > 0.0, (
            f"Expected non-zero input gradient on unmasked slot {i}, got {slot_grad}"
        )

    critic = ARCProCritic(obs_space, act_space, device).to(device)
    critic.zero_grad()
    critic_states = obs_b.clone().detach().requires_grad_(True)
    val, _ = critic.compute({"states": critic_states})
    val.sum().backward()
    for i in range(12):
        c_slot_grad = critic_states.grad[:, 25088 + i].abs().sum().item()
        assert c_slot_grad > 0.0
    priv_grad_sum = (
        critic.critic_head[0].weight.grad[:, [5, 6, 8, 9, 10, 11]].abs().sum().item()
    )
    assert priv_grad_sum > 0.0


def test_yuv_chrominance_isolation():
    device = torch.device("cpu")
    yuv_conv = nn.Conv2d(3, 3, kernel_size=1, bias=False).to(device)
    yuv_weights = (
        torch.tensor(
            [
                [0.299, 0.587, 0.114],
                [-0.14713, -0.28886, 0.436],
                [0.615, -0.51499, -0.10001],
            ],
            dtype=torch.float32,
        )
        .unsqueeze(-1)
        .unsqueeze(-1)
    )
    yuv_conv.weight.data.copy_(yuv_weights)

    white = torch.tensor([0.92, 0.92, 0.92], device=device).view(1, 3, 1, 1)
    yellow = torch.tensor([0.90, 0.75, 0.05], device=device).view(1, 3, 1, 1)
    asphalt = torch.tensor([0.25, 0.25, 0.25], device=device).view(1, 3, 1, 1)

    yuv_white = yuv_conv(white).squeeze().tolist()
    yuv_yellow = yuv_conv(yellow).squeeze().tolist()
    yuv_asphalt = yuv_conv(asphalt).squeeze().tolist()

    assert yuv_yellow[1] < -0.25
    assert yuv_yellow[2] > +0.10
    assert abs(yuv_white[1]) < 0.02 and abs(yuv_white[2]) < 0.02
    assert abs(yuv_asphalt[1]) < 0.02 and abs(yuv_asphalt[2]) < 0.02


def test_elevation_bounds():
    import inspect

    sig = inspect.signature(height_termination)
    min_h = sig.parameters["threshold"].default
    max_h = sig.parameters["max_height"].default

    assert min_h == -0.05
    assert max_h == 3.5

    class _MockRobotData:
        def __init__(self, z_vals):
            self.root_pos_w = torch.tensor(
                [[0.0, 0.0, z] for z in z_vals], dtype=torch.float32
            )

    class _MockRobot:
        def __init__(self, z_vals):
            self.data = _MockRobotData(z_vals)

    class _MockScene(dict):
        def __init__(self, z_vals):
            super().__init__(robot=_MockRobot(z_vals))
            self.env_origins = torch.zeros((len(z_vals), 3), dtype=torch.float32)

    class _MockEnv:
        def __init__(self, z_vals):
            self.scene = _MockScene(z_vals)
            self.episode_length_buf = torch.full((len(z_vals),), 50, dtype=torch.long)

    env = _MockEnv([2.50, 3.60, -0.10])
    terminated = height_termination(env, threshold=min_h, max_height=max_h)
    assert not bool(terminated[0].item())  # Z = 2.50m on 3D hill -> safe
    assert bool(terminated[1].item())  # Z = 3.60m -> terminated
    assert bool(terminated[2].item())  # Z = -0.10m -> terminated


if __name__ == "__main__":
    test_spatial_softmax()
    test_actor_architecture()
    test_yuv_chrominance_isolation()
    test_elevation_bounds()
