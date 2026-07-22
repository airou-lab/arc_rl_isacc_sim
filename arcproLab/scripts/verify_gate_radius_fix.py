# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause
"""
Verification probe for docs/off-road-debug/01-gate-radius-hole-punching.md.

Finds real white-boundary points from the live TrackManager point cloud at
three controlled distances from the nearest real gate, teleports one robot
per env onto each, and checks whether roadmark_contact termination fires:

  - near  (dist_gate ~0.10m): inside the correct 0.20m immunity zone -> should
    stay immune (no termination).
  - mid   (dist_gate ~0.20-0.50m): outside the correct 0.20m zone, but inside
    the old buggy 0.50m zone -> should NOW terminate. This is the band the
    fix changes; termination here is the signal the patch worked.
  - far   (dist_gate > 0.60m): nowhere near a gate -> should terminate in
    both old and new code (sanity control).

Headless, num_envs=3, no camera, no training, no checkpoints written.
"""
import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Verify gate-zone radius fix.")
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


def find_point_at_band(white_xyz, dist_to_gate, lo, hi, pick="min"):
    mask = (dist_to_gate >= lo) & (dist_to_gate < hi)
    idx_pool = mask.nonzero(as_tuple=True)[0]
    if len(idx_pool) == 0:
        return None, None
    d = dist_to_gate[idx_pool]
    chosen = idx_pool[torch.argmin(d)] if pick == "min" else idx_pool[torch.argmax(d)]
    return white_xyz[chosen], dist_to_gate[chosen].item()


def main():
    env_cfg = ARCProEnvCfg()
    env_cfg.scene.num_envs = 3
    env_cfg.enable_cameras = False
    env_cfg.observations.visual = None
    env_cfg.scene.tiled_camera = None
    env_cfg.__post_init__()

    env = ManagerBasedRLEnv(cfg=env_cfg)
    env.reset()

    tm = get_track_manager(device=env.device)
    tm.ensure_synced()

    if tm.white_tensor is None or tm.gate_tensor is None:
        print("CRITICAL: white_tensor or gate_tensor not built -- cannot verify.", flush=True)
        env.close()
        simulation_app.close()
        return

    white_xy = tm.white_tensor[:, :2]
    gate_xy = tm.gate_tensor[:, :2]
    dist_to_gate = torch.cdist(white_xy, gate_xy).min(dim=1)[0]

    near_pt, near_d = find_point_at_band(tm.white_tensor, dist_to_gate, 0.0, GATE_ZONE_RADIUS_M, pick="max")
    mid_pt, mid_d = find_point_at_band(tm.white_tensor, dist_to_gate, GATE_ZONE_RADIUS_M, 0.50, pick="min")
    far_pt, far_d = find_point_at_band(tm.white_tensor, dist_to_gate, 0.60, 1e9, pick="min")

    labels = ["near (<0.20m, should stay immune)",
              "mid  (0.20-0.50m, SHOULD NOW TERMINATE -- this is the fix)",
              "far  (>0.60m, control, should terminate)"]
    points = [near_pt, mid_pt, far_pt]
    dists = [near_d, mid_d, far_d]

    for lbl, pt, d in zip(labels, points, dists):
        status = f"found @ dist_gate={d:.3f}m" if pt is not None else "NO POINT FOUND IN BAND"
        print(f"[SETUP] {lbl}: {status}", flush=True)

    if any(p is None for p in points):
        print("CRITICAL: could not find test points in one or more distance bands; aborting.", flush=True)
        env.close()
        simulation_app.close()
        return

    # Teleport env i's robot onto points[i], stationary, safe drop height.
    env_origins = env.scene.env_origins
    pos = torch.zeros((3, 3), device=env.device)
    for i, pt in enumerate(points):
        pos[i, 0] = pt[0] + env_origins[i, 0]
        pos[i, 1] = pt[1] + env_origins[i, 1]
        pos[i, 2] = 0.15
    quat = torch.tensor([[0.7071, 0.0, 0.0, 0.7071]] * 3, device=env.device)
    env.scene["robot"].write_root_pose_to_sim(torch.cat([pos, quat], dim=-1))
    env.scene["robot"].write_root_velocity_to_sim(torch.zeros((3, 6), device=env.device))

    zero_action = torch.zeros((3, 3), device=env.device)  # stationary: steer=0, throttle=0, brake=0
    terminated_ever = torch.zeros(3, dtype=torch.bool)
    for step in range(30):
        obs, rewards, terminated, truncated, info = env.step(zero_action)
        terminated_ever |= terminated.cpu()
        if step == 0:
            # re-assert pose in case step 0 physics settle moved it before our check window
            pass

    print("\n--- RESULTS ---", flush=True)
    expected = [False, True, True]
    ok = True
    for lbl, d, term, exp in zip(labels, dists, terminated_ever.tolist(), expected):
        status = "OK" if term == exp else "FAIL"
        ok &= (term == exp)
        print(f"  [{status}] {lbl} (dist_gate={d:.3f}m): terminated={term} (expected={exp})", flush=True)

    print("\n[PASS] Gate-zone radius fix verified against live TrackManager data." if ok
          else "\n[FAIL] Live verification did not match expected behavior -- see above.", flush=True)

    env.close()
    simulation_app.close()


if __name__ == "__main__":
    main()
