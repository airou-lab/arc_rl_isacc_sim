# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause
"""
Runtime spawn yaw check (OM 3.3-A).

Launches the env headless, calls reset(), reads root_quat_w[0], computes yaw
from the (w, x, y, z) quaternion and prints it in radians and degrees. The
sim's reset event now targets +pi/2 (~1.5708 rad) to conform with the policy
repo's IsaacDirectConfig.spawn_yaw.
"""

import argparse
import math

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Verify spawn yaw after reset.")
parser.add_argument("--num_envs", type=int, default=1, help="Number of environments.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import os
import sys
import torch

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
ARCPRO_LAB_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, ".."))
if ARCPRO_LAB_DIR not in sys.path:
    sys.path.append(ARCPRO_LAB_DIR)

from isaaclab.envs import ManagerBasedRLEnv
from arcpro_env_cfg import ARCProEnvCfg


def main():
    env_cfg = ARCProEnvCfg()
    env_cfg.scene.num_envs = args_cli.num_envs
    env_cfg.enable_cameras = False
    env_cfg.__post_init__()

    env = ManagerBasedRLEnv(cfg=env_cfg)
    env.reset()

    asset = env.scene["robot"]
    q = asset.data.root_quat_w[0]
    qw, qx, qy, qz = q[0].item(), q[1].item(), q[2].item(), q[3].item()
    yaw_rad = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
    yaw_deg = math.degrees(yaw_rad)

    print("=" * 60)
    print(f"Spawn yaw (env 0): {yaw_rad:+.4f} rad ({yaw_deg:+.2f} deg)")
    print(f"Expected: +1.5708 rad (+90.00 deg) per OM 3.3-A")
    print("=" * 60)

    env.close()
    simulation_app.close()


if __name__ == "__main__":
    main()
