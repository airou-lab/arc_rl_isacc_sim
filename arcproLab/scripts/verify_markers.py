# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""
Interactive Visual Semantic Audit.
Robot auto-crawls forward to test stop-line resets.
"""

import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--num_envs", type=int, default=1)
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import os
import sys
sys.path.append(os.path.join(os.path.dirname(__file__), ".."))

import torch
from isaaclab.envs import ManagerBasedRLEnv
from arcpro_env_cfg import ARCProEnvCfg
from mdp.track_manager import get_track_manager

def main():
    env_cfg = ARCProEnvCfg()
    env_cfg.scene.num_envs = 1
    env_cfg.enable_cameras = True 
    env = ManagerBasedRLEnv(cfg=env_cfg)

    tm = get_track_manager(device=env.device)
    
    print("\n" + "="*50)
    print("ARCPro INTERACTIVE Audit Active")
    print("ROBOT IS AUTO-CRAWLING FORWARD")
    print("RED: Boundaries | GREEN: Intersections | CYAN: Path")
    print("="*50 + "\n")

    # Simple loop - no complex keyboard for now to avoid crashes
    while simulation_app.is_running():
        # [Steer, Throttle, Brake]
        # Crawl at 0.3 throttle to test lines
        actions = torch.zeros((env.num_envs, 3), device=env.device)
        actions[:, 1] = 0.3 
        
        env.step(actions)
        tm.refresh_visuals()

    env.close()

if __name__ == "__main__":
    main()
