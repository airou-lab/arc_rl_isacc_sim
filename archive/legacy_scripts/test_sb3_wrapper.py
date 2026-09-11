import os
import sys
import argparse

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, "..", ".."))
ARCPRO_LAB_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, ".."))
if ROOT_DIR not in sys.path:
    sys.path.insert(0, ROOT_DIR)
if ARCPRO_LAB_DIR not in sys.path:
    sys.path.insert(0, ARCPRO_LAB_DIR)

from isaaclab.app import AppLauncher
parser = argparse.ArgumentParser()
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

from isaaclab.envs import ManagerBasedRLEnv
from arcpro_env_cfg import ARCProEnvCfg
from isaaclab_rl.sb3 import Sb3VecEnvWrapper

env_cfg = ARCProEnvCfg()
env_cfg.scene.num_envs = 1
env_cfg.enable_cameras = True 
env_cfg.__post_init__() 

env = ManagerBasedRLEnv(cfg=env_cfg)
print("Original obs space:", env.observation_space)
print("Single obs space:", env.single_observation_space)

try:
    sb3_env = Sb3VecEnvWrapper(env)
    print("Sb3VecEnvWrapper (default) obs space:", sb3_env.observation_space)
except Exception as e:
    print("Sb3VecEnvWrapper (default) failed:", e)

try:
    sb3_env2 = Sb3VecEnvWrapper(env, obs_dict_keys=["policy", "visual"])
    print("Sb3VecEnvWrapper (dict) obs space:", sb3_env2.observation_space)
except Exception as e:
    print("Sb3VecEnvWrapper (dict) failed:", e)

simulation_app.close()
