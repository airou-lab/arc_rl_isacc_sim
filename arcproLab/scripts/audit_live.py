# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import argparse
from isaaclab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(description="Audit live simulation stage.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import os
import sys
import torch

# Add arcproLab to sys.path
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
ARCPRO_LAB_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, ".."))
if ARCPRO_LAB_DIR not in sys.path:
    sys.path.append(ARCPRO_LAB_DIR)

from isaaclab.envs import ManagerBasedRLEnv
from arcpro_env_cfg import ARCProEnvCfg
import omni.usd
from pxr import Usd, UsdGeom

def main():
    # 1. Setup Environment
    env_cfg = ARCProEnvCfg()
    env_cfg.scene.num_envs = 1
    # Disable cameras for faster audit
    env_cfg.enable_cameras = False
    env_cfg.observations.visual = None
    env_cfg.__post_init__()

    # 2. Create Environment (This spawns the robot)
    env = ManagerBasedRLEnv(cfg=env_cfg)
    
    # 3. Traverse Live Stage
    stage = omni.usd.get_context().get_stage()
    print("\n--- Traversing Live Stage ---")
    
    for prim in stage.Traverse():
        path = str(prim.GetPath())
        if "Robot" in path or "Track" in path:
            if prim.IsA(UsdGeom.Mesh) or prim.IsA(UsdGeom.Xform):
                print(f"[{prim.GetTypeName()}] {path}")

    env.close()
    simulation_app.close()

if __name__ == "__main__":
    main()
