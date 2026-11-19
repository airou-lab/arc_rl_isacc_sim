#!/usr/bin/env python3
"""
Audit Gate 1: Wrapper & Replay Buffer Pointer Non-Aliasing (s_t vs s_{t+1})
Verification of SKRLFlattenWrapper ping-pong buffer management and SKRL RandomMemory storage.
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

from agents.skrl_wrappers import SKRLFlattenWrapper  # noqa: E402
from skrl.memories.torch import RandomMemory  # noqa: E402

pytestmark = pytest.mark.cpu


class MockSensorOutput:
    def __init__(self, num_envs=4, device="cpu"):
        self.output = {
            "rgb": torch.randint(
                0, 255, (num_envs, 224, 224, 3), dtype=torch.uint8, device=device
            )
        }


class MockSensor:
    def __init__(self, num_envs=4, device="cpu"):
        self.data = MockSensorOutput(num_envs, device)


class MockScene:
    def __init__(self, num_envs=4, device="cpu"):
        self.sensors = {"tiled_camera": MockSensor(num_envs, device)}

    def __getitem__(self, key):
        raise KeyError(key)


class MockCfg:
    enable_cameras = True


class MockIsaacEnv:
    """Mock IsaacLab environment returning structured telemetry and camera frames."""

    def __init__(self, num_envs=4, device="cpu"):
        self.cfg = MockCfg()
        self.num_envs = num_envs
        self.device = torch.device(device)
        self.scene = MockScene(num_envs, device)
        self.observation_space = gym.spaces.Box(-float("inf"), float("inf"), (12,))
        self.action_space = gym.spaces.Box(-1.0, 1.0, (2,))
        self.extras = {}
        self.step_count = 0

    @property
    def unwrapped(self):
        return self

    def reset(self):
        self.step_count = 0
        telemetry = torch.full((self.num_envs, 12), fill_value=0.1, device=self.device)
        obs = {"telemetry": telemetry}
        return obs, {}

    def step(self, actions):
        self.step_count += 1
        fill_val = 1.0 + 0.5 * self.step_count
        telemetry = torch.full(
            (self.num_envs, 12), fill_value=fill_val, device=self.device
        )
        obs = {"telemetry": telemetry}
        reward = torch.ones(self.num_envs, device=self.device)
        terminated = torch.zeros(self.num_envs, dtype=torch.bool, device=self.device)
        truncated = torch.zeros(self.num_envs, dtype=torch.bool, device=self.device)
        info = {}
        return obs, reward, terminated, truncated, info


def test_ping_pong_buffer_allocation_and_toggling():
    device = "cpu"
    num_envs = 4
    base_env = MockIsaacEnv(num_envs=num_envs, device=device)
    wrapper = SKRLFlattenWrapper(base_env)

    assert not hasattr(wrapper, "_flat_obs_buffers")

    obs_0, _ = wrapper.reset()
    assert hasattr(wrapper, "_flat_obs_buffers")
    assert len(wrapper._flat_obs_buffers) == 2
    assert wrapper._buf_idx == 1

    ptr_buf0 = wrapper._flat_obs_buffers[0].data_ptr()
    ptr_buf1 = wrapper._flat_obs_buffers[1].data_ptr()
    ptr_obs0 = obs_0.data_ptr()

    assert ptr_buf0 != ptr_buf1
    assert ptr_obs0 == ptr_buf1

    obs_0_telemetry_snapshot = obs_0[:, 25088:].clone()

    actions = torch.zeros((num_envs, 2), device=device)
    obs_1, rew, term, trunc, info = wrapper.step(actions)
    ptr_obs1 = obs_1.data_ptr()

    assert wrapper._buf_idx == 0
    assert ptr_obs1 == ptr_buf0
    assert ptr_obs0 != ptr_obs1
    assert torch.equal(obs_0[:, 25088:], obs_0_telemetry_snapshot)

    obs_2, _, _, _, _ = wrapper.step(actions)
    ptr_obs2 = obs_2.data_ptr()
    assert wrapper._buf_idx == 1
    assert ptr_obs2 == ptr_buf1
    assert ptr_obs2 == ptr_obs0
    assert ptr_obs2 != ptr_obs1


def test_skrl_rollout_replay_buffer_non_aliasing():
    device = "cpu"
    num_envs = 4
    base_env = MockIsaacEnv(num_envs=num_envs, device=device)
    wrapper = SKRLFlattenWrapper(base_env)

    memory_size = 16
    memory = RandomMemory(memory_size=memory_size, num_envs=num_envs, device="cpu")
    memory.create_tensor(name="states", size=(25100,), dtype=torch.float32)
    memory.create_tensor(name="next_states", size=(25100,), dtype=torch.float32)
    memory.create_tensor(name="actions", size=(2,), dtype=torch.float32)
    memory.create_tensor(name="rewards", size=(1,), dtype=torch.float32)

    states, _ = wrapper.reset()
    assert wrapper._buf_idx == 1

    for step in range(5):
        actions = torch.full((num_envs, 2), fill_value=0.1 * step, device=device)
        pre_step_states_ptr = states.data_ptr()
        pre_step_states_val = states[:, 25088:].clone()

        next_states, rewards, term, trunc, info = wrapper.step(actions)

        assert states.data_ptr() != next_states.data_ptr()
        assert states.data_ptr() == pre_step_states_ptr
        assert torch.equal(states[:, 25088:], pre_step_states_val)

        memory.add_samples(
            states=states.cpu(),
            next_states=next_states.cpu(),
            actions=actions.cpu(),
            rewards=rewards.unsqueeze(-1).cpu(),
        )

        recorded_states = memory.tensors["states"][step]
        recorded_next_states = memory.tensors["next_states"][step]
        assert not torch.equal(
            recorded_states[:, 25088:], recorded_next_states[:, 25088:]
        )

        states = next_states


def test_reset_step_behavior():
    device = "cpu"
    num_envs = 4
    base_env = MockIsaacEnv(num_envs=num_envs, device=device)
    wrapper = SKRLFlattenWrapper(base_env)

    states_0, _ = wrapper.reset()
    ptr_0 = states_0.data_ptr()

    actions = torch.zeros((num_envs, 2), device=device)
    states_1, _, _, _, _ = wrapper.step(actions)
    ptr_1 = states_1.data_ptr()

    states_reset, _ = wrapper.reset()
    ptr_reset = states_reset.data_ptr()

    assert ptr_reset != ptr_1
    assert ptr_reset == ptr_0


def test_contrast_single_static_buffer_failure_mode():
    device = "cpu"
    num_envs = 4
    single_static_buffer = torch.zeros((num_envs, 25100), device=device)
    single_static_buffer[:, 25088:] = 1.0
    s_t = single_static_buffer
    single_static_buffer[:, 25088:] = 2.0
    s_next = single_static_buffer
    assert s_t.data_ptr() == s_next.data_ptr()
    assert s_t[0, 25088].item() == 2.0


if __name__ == "__main__":
    test_ping_pong_buffer_allocation_and_toggling()
    test_skrl_rollout_replay_buffer_non_aliasing()
    test_reset_step_behavior()
    test_contrast_single_static_buffer_failure_mode()
