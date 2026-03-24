import os
import sys
import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
app = AppLauncher(args).app

# Setup paths
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, ".."))
if ROOT_DIR not in sys.path:
    sys.path.append(ROOT_DIR)

import torch
from isaaclab.envs import ManagerBasedEnv
from arcpro_env_cfg import ARCProEnvCfg

env_cfg = ARCProEnvCfg()
env_cfg.scene.num_envs = 1
env_cfg.__post_init__() 

# Use Final_Fixed.usd which worked before
env_cfg.scene.robot.spawn.usd_path = "/home/arika/Documents/arcpro/arcpro_system/src/examples/ARCPro_RL/arc_rl_isacc_sim/f1tenth_trainer/assets/F1Tenth_Final_Fixed.usd"

env = ManagerBasedEnv(cfg=env_cfg)
robot = env.scene["robot"]

print("\n--- Articulation Internal Audit ---")
print(f"Num joints: {robot.num_joints}")
print(f"Joint names: {robot.data.joint_names}")

app.close()
