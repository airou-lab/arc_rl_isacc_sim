#!/usr/bin/env python3
"""
Unit test to verify that TelemetryPPO._update executes proper backward passes
with decoupled policy/value optimizers and updates weights on mini-batches
without premature early-stopping freezing.
"""

import os
import sys

import gymnasium as gym
import pytest
import torch

WORKSPACE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if WORKSPACE_DIR not in sys.path:
    sys.path.insert(0, WORKSPACE_DIR)
sys.path.insert(0, os.path.join(WORKSPACE_DIR, "arcproLab"))
sys.path.insert(0, os.path.join(WORKSPACE_DIR, "arcproLab", "scripts"))

from agents.skrl_models import ARCProActor, ARCProCritic  # noqa: E402
from agents.skrl_wrappers import SKRLFlattenWrapper  # noqa: E402
from agents.telemetry_ppo import PPO_DEFAULT_CONFIG, TelemetryPPO  # noqa: E402
from skrl.memories.torch import RandomMemory  # noqa: E402

pytestmark = pytest.mark.cpu


def test_telemetry_ppo_gradient_step():
    device = "cpu"
    obs_space = gym.spaces.Box(low=-float("inf"), high=float("inf"), shape=(25100,))
    act_space = gym.spaces.Box(low=-1.0, high=1.0, shape=(2,))

    actor = ARCProActor(obs_space, act_space, device).to(device)
    critic = ARCProCritic(obs_space, act_space, device).to(device)

    models = {"policy": actor, "value": critic}

    num_envs = 4
    rollouts = 16
    mini_batches = 2

    memory = RandomMemory(memory_size=rollouts, num_envs=num_envs, device="cpu")

    cfg_ppo = PPO_DEFAULT_CONFIG.copy()
    cfg_ppo["rollouts"] = rollouts
    cfg_ppo["mini_batches"] = mini_batches
    cfg_ppo["learning_epochs"] = 2
    cfg_ppo["learning_rate"] = 1e-3
    cfg_ppo["random_timesteps"] = 0
    cfg_ppo["learning_starts"] = 0
    cfg_ppo["state_preprocessor"] = None
    cfg_ppo["kl_threshold"] = 0.016
    cfg_ppo["entropy_loss_scale"] = 0.005
    cfg_ppo["learning_rate_scheduler"] = None

    agent = TelemetryPPO(
        models=models,
        memory=memory,
        cfg=cfg_ppo,
        observation_space=obs_space,
        action_space=act_space,
        device=device,
        initial_step=0,
        num_envs=num_envs,
    )

    agent.init()
    agent.set_mode("train")

    assert hasattr(agent, "policy_optimizer") and hasattr(agent, "value_optimizer")
    assert agent.policy_optimizer is not agent.value_optimizer

    for _ in range(rollouts):
        states = torch.randn(num_envs, 25100, device="cpu")
        actions = torch.clamp(torch.randn(num_envs, 2, device="cpu"), -1.0, 1.0)
        rewards = torch.ones(num_envs, 1, device="cpu") * 5.0
        next_states = torch.randn(num_envs, 25100, device="cpu")
        terminated = torch.zeros(num_envs, 1, dtype=torch.bool, device="cpu")
        truncated = torch.zeros(num_envs, 1, dtype=torch.bool, device="cpu")
        log_prob = torch.zeros(num_envs, 1, device="cpu")
        values = torch.zeros(num_envs, 1, device="cpu")

        memory.add_samples(
            states=states,
            actions=actions,
            rewards=rewards,
            next_states=next_states,
            terminated=terminated,
            truncated=truncated,
            log_prob=log_prob,
            values=values,
        )

    agent._current_next_states = torch.randn(num_envs, 25100, device=device)

    initial_policy_weights = {k: v.clone() for k, v in actor.state_dict().items()}
    initial_value_weights = {k: v.clone() for k, v in critic.state_dict().items()}

    agent._update(timestep=16, timesteps=1000)

    policy_loss_recorded = agent.tracking_data.get("Loss / Policy loss", [])
    value_loss_recorded = agent.tracking_data.get("Loss / Value loss", [])

    assert len(policy_loss_recorded) > 0, "No policy loss was recorded!"
    assert len(value_loss_recorded) > 0, "No value loss was recorded!"
    assert policy_loss_recorded[-1] != 0.0, "Policy loss is exactly 0.0!"
    assert value_loss_recorded[-1] != 0.0, "Value loss is exactly 0.0!"

    max_policy_diff = max(
        (actor.state_dict()[k] - initial_policy_weights[k]).abs().max().item()
        for k in initial_policy_weights
    )
    max_value_diff = max(
        (critic.state_dict()[k] - initial_value_weights[k]).abs().max().item()
        for k in initial_value_weights
    )

    assert max_policy_diff > 0.0, "Policy weights did not update! Max diff is 0.0"
    assert max_value_diff > 0.0, "Value weights did not update! Max diff is 0.0"


def test_Critic_privileged_gradient_and_ping_pong_buffers():
    device = "cpu"
    obs_space = gym.spaces.Box(low=-float("inf"), high=float("inf"), shape=(25100,))
    act_space = gym.spaces.Box(low=-1.0, high=1.0, shape=(2,))

    actor = ARCProActor(obs_space, act_space, device).to(device)
    critic = ARCProCritic(obs_space, act_space, device).to(device)

    num_envs = 4
    rollouts = 8
    memory = RandomMemory(memory_size=rollouts, num_envs=num_envs, device="cpu")

    cfg_ppo = PPO_DEFAULT_CONFIG.copy()
    cfg_ppo["rollouts"] = rollouts
    cfg_ppo["mini_batches"] = 1
    cfg_ppo["learning_epochs"] = 1
    cfg_ppo["learning_rate"] = 1e-3
    cfg_ppo["state_preprocessor"] = None
    cfg_ppo["kl_threshold"] = 0.0
    cfg_ppo["learning_rate_scheduler"] = None

    agent = TelemetryPPO(
        models={"policy": actor, "value": critic},
        memory=memory,
        cfg=cfg_ppo,
        observation_space=obs_space,
        action_space=act_space,
        device=device,
        initial_step=0,
        num_envs=num_envs,
    )
    agent.init()
    agent.set_mode("train")

    for i in range(rollouts):
        states = torch.zeros(num_envs, 25100, device="cpu")
        states[:, 25088 + 8] = 0.25 * (i + 1)  # lat_err
        states[:, 25088 + 9] = -0.35 * (i + 1)  # head_err
        states[:, 25088 + 11] = 1.50 * (i + 1)  # distance
        actions = torch.zeros(num_envs, 2, device="cpu")
        rewards = torch.ones(num_envs, 1, device="cpu") * 10.0
        next_states = states.clone()
        terminated = torch.zeros(num_envs, 1, dtype=torch.bool, device="cpu")
        truncated = torch.zeros(num_envs, 1, dtype=torch.bool, device="cpu")
        log_prob = torch.zeros(num_envs, 1, device="cpu")
        values = torch.zeros(num_envs, 1, device="cpu")

        memory.add_samples(
            states=states,
            actions=actions,
            rewards=rewards,
            next_states=next_states,
            terminated=terminated,
            truncated=truncated,
            log_prob=log_prob,
            values=values,
        )

    agent._current_next_states = torch.zeros(num_envs, 25100, device=device)
    init_priv_weights = critic.critic_head[0].weight[:, [8, 9, 11]].detach().clone()

    agent._update(timestep=rollouts, timesteps=100)

    assert critic.critic_head[0].weight.grad is not None, "Critic weight.grad is None!"
    priv_grad_norm = critic.critic_head[0].weight.grad[:, [8, 9, 11]].abs().sum().item()
    priv_weight_diff = (
        (critic.critic_head[0].weight[:, [8, 9, 11]] - init_priv_weights)
        .abs()
        .sum()
        .item()
    )
    assert priv_grad_norm > 0.0, (
        f"Critic privileged slots [8, 9, 11] received zero gradient ({priv_grad_norm})!"
    )
    assert priv_weight_diff > 0.0, (
        f"Critic privileged weights [8, 9, 11] did not update ({priv_weight_diff})!"
    )

    class _DummyCamData:
        def __init__(self, rgb):
            self.output = {"rgb": rgb}

    class _DummyCam:
        def __init__(self, rgb):
            self.data = _DummyCamData(rgb)

    class _DummyScene:
        def __init__(self, rgb):
            self.sensors = {"tiled_camera": _DummyCam(rgb)}

    class _DummyCfg:
        enable_cameras = True

    class _DummyEnv:
        def __init__(self, num_envs, device):
            self.cfg = _DummyCfg()
            self.num_envs = num_envs
            self.device = device
            self.scene = _DummyScene(
                torch.randint(
                    0, 255, (num_envs, 224, 224, 3), device=device, dtype=torch.uint8
                )
            )
            self.unwrapped = self
            self.observation_space = obs_space
            self.action_space = act_space
            self.state_space = obs_space

    dummy_env = _DummyEnv(num_envs=2, device=device)
    wrapper = SKRLFlattenWrapper(dummy_env)

    tel_t0 = torch.full((2, 12), 1.0, device=device)
    tel_t1 = torch.full((2, 12), 2.0, device=device)

    buf0 = wrapper._flatten_obs({"policy": tel_t0})
    buf1 = wrapper._flatten_obs({"policy": tel_t1})

    assert buf0.data_ptr() != buf1.data_ptr(), (
        "SKRLFlattenWrapper returned identical tensor storage on consecutive calls (in-place aliasing bug)!"
    )
    assert torch.allclose(buf0[:, -12:], tel_t0)
    assert torch.allclose(buf1[:, -12:], tel_t1)


if __name__ == "__main__":
    test_telemetry_ppo_gradient_step()
    test_Critic_privileged_gradient_and_ping_pong_buffers()
