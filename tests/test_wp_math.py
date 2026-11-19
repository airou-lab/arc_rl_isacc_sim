#!/usr/bin/env python3
"""Waypoint indexing, heading alignment, track_dir, and cumulative progress math verification."""

import math
import os
import sys

import pytest
import torch

ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ARCPRO_LAB_DIR = os.path.join(ROOT_DIR, "arcproLab")
for _p in (ROOT_DIR, ARCPRO_LAB_DIR):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from arcproLab.mdp.rewards import waypoint_progress_reward
from arcproLab.mdp.track_manager import get_track_manager
from arcproLab.mdp.track_topology import SECTOR_DEFS, compute_step_road_state

pytestmark = pytest.mark.cpu


class _MockRobotData:
    def __init__(self, pos: torch.Tensor, quat: torch.Tensor):
        self.root_pos_w = pos
        self.root_quat_w = quat
        self.root_lin_vel_b = torch.tensor([[-0.50, 0.0, 0.0]], dtype=torch.float32)
        self.root_ang_vel_b = torch.zeros((1, 3), dtype=torch.float32)


class _MockRobot:
    def __init__(self, pos: torch.Tensor, quat: torch.Tensor):
        self.data = _MockRobotData(pos, quat)


class _MockScene:
    def __init__(self, pos: torch.Tensor, quat: torch.Tensor):
        self.num_envs = 1
        self.env_origins = torch.zeros((1, 3), dtype=torch.float32)
        self._robot = _MockRobot(pos, quat)

    def __getitem__(self, key: str):
        if key == "robot":
            return self._robot
        raise KeyError(key)

    def get(self, key: str, default=None):
        return self._robot if key == "robot" else default


class _MockEnv:
    def __init__(self, pos: torch.Tensor, quat: torch.Tensor):
        self.num_envs = 1
        self.device = "cpu"
        self.step_dt = 0.02
        self.common_step_counter = 0
        self.episode_length_buf = torch.tensor([10], dtype=torch.long)
        self.reset_terminated = torch.tensor([False], dtype=torch.bool)
        self.reset_buf = torch.tensor([False], dtype=torch.bool)
        self.extras = {}
        self.scene = _MockScene(pos, quat)


def test_wp_math_forward_progress():
    """Verify waypoint index, yaw error, track_dir=+1.0, and cumulative_wp_index across 20 forward steps."""
    tm = get_track_manager(device="cpu")
    tm.ensure_synced()

    s1_x, s1_y, s1_yaw = SECTOR_DEFS[1]
    half_yaw = s1_yaw / 2.0
    quat = torch.tensor(
        [[math.cos(half_yaw), 0.0, 0.0, math.sin(half_yaw)]], dtype=torch.float32
    )
    pos = torch.tensor([[s1_x, s1_y, 0.12]], dtype=torch.float32)

    env = _MockEnv(pos, quat)
    tm.reset_indices(env_ids=torch.tensor([0]), new_pos=pos[:, :2])

    state0 = compute_step_road_state(env)
    _ = waypoint_progress_reward(env)
    assert state0["track_dir"].item() == 1.0
    assert abs(state0["head_err"].item()) < 0.05

    prev_cum_wp = 0.0
    for step_idx in range(1, 21):
        env.common_step_counter = step_idx
        env.episode_length_buf = torch.tensor([10 + step_idx], dtype=torch.long)
        # Advance 0.015m South along the West straightaway per step
        pos[0, 1] -= 0.015
        state = compute_step_road_state(env)
        rew_delta = waypoint_progress_reward(env)

        wps_cum = float(env.extras["cumulative_wp_index"][0].item())
        assert state["track_dir"].item() == 1.0
        assert abs(state["head_err"].item()) < 0.05
        assert abs(state["lat_err"].item()) < 0.05
        assert wps_cum >= prev_cum_wp
        assert rew_delta[0].item() >= 0.0
        prev_cum_wp = wps_cum

    assert prev_cum_wp > 15.0, f"Expected cumulative WP progress > 15, got {prev_cum_wp}"


if __name__ == "__main__":
    test_wp_math_forward_progress()
    print("[PASSED] test_wp_math_forward_progress")
