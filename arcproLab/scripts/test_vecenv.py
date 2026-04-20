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
import gymnasium as gym

parser = argparse.ArgumentParser()
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
app_launcher = AppLauncher(args_cli)

from isaaclab.envs import ManagerBasedRLEnv
from arcpro_env_cfg import ARCProEnvCfg

env_cfg = ARCProEnvCfg()
env_cfg.scene.num_envs = 1
env_cfg.enable_cameras = True 
env_cfg.__post_init__() 

env = ManagerBasedRLEnv(cfg=env_cfg)

print("Obs space:", env.observation_space)
print("Action space:", env.action_space)
print("Single Obs space:", env.single_observation_space)
print("Single Action space:", env.single_action_space)

from stable_baselines3.common.vec_env import VecEnv
class TestVecEnv(VecEnv):
    def __init__(self):
        obs_space = gym.spaces.Dict({
            "vec": env.single_observation_space["policy"],
            "image": env.single_observation_space["visual"]
        })
        act_space = env.single_action_space
        print("TestVecEnv Obs space type:", type(obs_space))
        print("TestVecEnv Act space type:", type(act_space))
        super().__init__(1, obs_space, act_space)
    def reset(self): pass
    def step_async(self, a): pass
    def step_wait(self): pass
    def close(self): pass
    def get_attr(self, a, i=None): 
        val = getattr(env, a, None)
        return [val] * 1
    def set_attr(self, a, v, i=None): pass
    def env_method(self, m, *a, **k): pass
    def env_is_wrapped(self, w, i=None): return [False]

try:
    TestVecEnv()
    print("TestVecEnv initialized successfully")
except Exception as e:
    import traceback
    traceback.print_exc()

app_launcher.app.close()
