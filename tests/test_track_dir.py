#!/usr/bin/env python3
"""Pure-CPU verification of TrackManager waypoint direction and Sector 1 heading."""

import os
import sys

import pytest
import torch
from scipy.spatial.transform import Rotation

ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ARCPRO_LAB_DIR = os.path.join(ROOT_DIR, "arcproLab")
for _p in (ROOT_DIR, ARCPRO_LAB_DIR):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from mdp.track_manager import TrackManager  # noqa: E402
from mdp.track_topology import SECTOR_DEFS  # noqa: E402

pytestmark = pytest.mark.cpu


class MockEnv:
    def __init__(self):
        self.device = "cpu"
        self.num_envs = 1


def test_track_dir_and_nearest_waypoint():
    env = MockEnv()
    tm = TrackManager(env)
    sec1_x, sec1_y, _sec1_yaw = SECTOR_DEFS[1]
    pos = torch.tensor([[sec1_x, sec1_y, 0.5]], dtype=torch.float32)
    wp_idx = tm.get_nearest_waypoint(pos)
    wp_pos = tm.waypoints[wp_idx[0]]
    next_wp_idx = (wp_idx[0] + 1) % len(tm.waypoints)
    next_wp_pos = tm.waypoints[next_wp_idx]
    vec = next_wp_pos - wp_pos

    # Start line (Sector 1) heads South (-Y direction)
    assert vec[1].item() < 0.0, f"Expected South (-Y) tangent at Sector 1, got {vec}"

    # Verify scipy Rotation for (w=0.7071, x=0, y=0, z=-0.7071) -> scipy [x, y, z, w] = [0, 0, -0.7071, 0.7071]
    r = Rotation.from_quat([0.0, 0.0, -0.7071068, 0.7071068])
    heading = r.apply([1.0, 0.0, 0.0])
    assert abs(heading[0]) < 1e-4 and abs(heading[1] - (-1.0)) < 1e-4


if __name__ == "__main__":
    test_track_dir_and_nearest_waypoint()
