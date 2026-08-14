import sys
import os
import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args(["--enable_cameras", "--headless"])
app_launcher = AppLauncher(args_cli)
app_launcher.app

import torch
from isaaclab.envs import ManagerBasedRLEnv
from arcproLab.arcpro_env_cfg import ARCProEnvCfg

env_cfg = ARCProEnvCfg()
env_cfg.scene.num_envs = 1
env_cfg.enable_cameras = True
env_cfg.__post_init__()

env = ManagerBasedRLEnv(cfg=env_cfg)
print("\n--- ACTIVE OBSERVATION TERMS ---")
for group_name, group in env.observation_manager.active_terms.items():
    print(f"Group: {group_name}")
    for term_name in group.keys():
        print(f"  Term: {term_name}")

env.close()
