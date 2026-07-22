# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause
"""
Single-agent, single-env sanity check for docs/off-road-debug/02-gate-discovery-env-scoping.md.

Boots num_envs=1 (no cross-env contamination possible even before the fix,
so this isolates whether a real DSLaneGate exists near the driven track at
all) and reports gate_tensor size plus distance from the spawn point and
from every track_centerline.npy waypoint to the nearest discovered gate.

Headless, num_envs=1, no camera, no training, no checkpoints written.
"""
import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Verify gate discovery is scoped correctly for a single env.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
args_cli.headless = True
args_cli.enable_cameras = False

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import os
import sys
import torch

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
ARCPRO_LAB_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, ".."))
if ARCPRO_LAB_DIR not in sys.path:
    sys.path.insert(0, ARCPRO_LAB_DIR)

from arcpro_env_cfg import ARCProEnvCfg
from isaaclab.envs import ManagerBasedRLEnv
from mdp.track_manager import get_track_manager, GATE_ZONE_RADIUS_M


def main():
    env_cfg = ARCProEnvCfg()
    env_cfg.scene.num_envs = 1
    env_cfg.enable_cameras = False
    env_cfg.observations.visual = None
    env_cfg.scene.tiled_camera = None
    env_cfg.__post_init__()

    env = ManagerBasedRLEnv(cfg=env_cfg)
    env.reset()

    tm = get_track_manager(device=env.device)
    tm.ensure_synced()

    print(f"[RESULT] white points: {0 if tm.white_tensor is None else len(tm.white_tensor)}", flush=True)
    print(f"[RESULT] yellow points: {0 if tm.yellow_tensor is None else len(tm.yellow_tensor)}", flush=True)
    print(f"[RESULT] gate points: {0 if tm.gate_tensor is None else len(tm.gate_tensor)}", flush=True)

    if tm.gate_tensor is None or len(tm.gate_tensor) == 0:
        print("[RESULT] NO GATES FOUND AT ALL for a single env -- track has no tagged DSLaneGate near it, or discovery is still broken.", flush=True)
        env.close()
        simulation_app.close()
        return

    spawn = env.scene["robot"].data.root_pos_w[0, :2] - env.scene.env_origins[0, :2]
    gate_xy = tm.gate_tensor[:, :2]
    d_spawn = torch.cdist(spawn.unsqueeze(0), gate_xy).min().item()
    print(f"[RESULT] spawn {spawn.cpu().tolist()} -> nearest gate: {d_spawn:.3f}m", flush=True)

    wp_path = os.path.join(ARCPRO_LAB_DIR, "mdp", "track_centerline.npy")
    if os.path.exists(wp_path):
        import numpy as np
        wp = np.load(wp_path)
        wp_xy = torch.tensor(wp[:, :2], device=env.device, dtype=torch.float32)
        d_centerline = torch.cdist(wp_xy, gate_xy).min(dim=1)[0]
        print(f"[RESULT] centerline-to-nearest-gate: min={d_centerline.min().item():.3f}m "
              f"max={d_centerline.max().item():.3f}m mean={d_centerline.mean().item():.3f}m", flush=True)
        near_count = (d_centerline < GATE_ZONE_RADIUS_M).sum().item()
        print(f"[RESULT] centerline waypoints within GATE_ZONE_RADIUS_M ({GATE_ZONE_RADIUS_M}m) of a gate: "
              f"{near_count} / {len(wp)}", flush=True)

    print("\n[PASS] Gate discovery found gate(s) near the single-env track." if d_spawn < 200
          else "\n[INFO] Gates found, but still far from track -- see numbers above.", flush=True)

    env.close()
    simulation_app.close()


if __name__ == "__main__":
    main()
