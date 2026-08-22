import torch
import numpy as np
import sys
sys.path.append("arcproLab")
from mdp.track_manager import TrackManager

class MockEnv:
    def __init__(self):
        self.device = "cpu"
        self.num_envs = 1

env = MockEnv()
tm = TrackManager(env)
pos = torch.tensor([[-16.197, 5.50, 0.5]])
wp_idx = tm.get_nearest_waypoint(pos)
wp_pos = tm.waypoints[wp_idx[0]]
next_wp_idx = (wp_idx[0] + 1) % len(tm.waypoints)
next_wp_pos = tm.waypoints[next_wp_idx]

print(f"Nearest WP index: {wp_idx[0]}")
print(f"WP pos: {wp_pos}")
print(f"Next WP pos: {next_wp_pos}")
vec = next_wp_pos - wp_pos
print(f"Direction vector: {vec}")

from scipy.spatial.transform import Rotation
r = Rotation.from_quat([-0.7071, 0.0, 0.0, 0.7071]) # x, y, z, w
print("Heading for (0.7071, 0, 0, -0.7071):", r.apply([1, 0, 0])) # Assuming car forward is +X
