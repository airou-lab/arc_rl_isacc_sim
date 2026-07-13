import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Play SKRL ARCPro Policy")
parser.add_argument("--checkpoint", type=str, default="logs/ppo_skrl/20260701-192503/26-07-01_19-25-03-529362_PPO/checkpoints/best_agent.pt", help="Path to SKRL checkpoint .pt file")
parser.add_argument("--num_envs", type=int, default=1, help="Number of parallel environments")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
args_cli.enable_cameras = True # Enable cameras for visuals
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import sys
import os

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, "..", ".."))
if ROOT_DIR not in sys.path:
    sys.path.insert(0, ROOT_DIR)

print("Importing torch...")
sys.stdout.flush()
import torch
import torch.nn as nn

print("Importing IsaacLab...")
sys.stdout.flush()
from isaaclab.envs import ManagerBasedRLEnv

print("Importing SKRL...")
sys.stdout.flush()
from skrl.agents.torch.ppo import PPO, PPO_DEFAULT_CONFIG

print("Importing ARCPro modules...")
sys.stdout.flush()
from arcproLab.arcpro_env_cfg import ARCProEnvCfg
from arcproLab.agents.skrl_models import ARCProActor, ARCProCritic
from skrl.envs.wrappers.torch import IsaacLabWrapper, Wrapper
import numpy as np
import gymnasium as gym

class SKRLFlattenWrapper(Wrapper):
    def __init__(self, env):
        super().__init__(env)
        
    @property
    def observation_space(self):
        shape_dim = 150540 if self._env.cfg.enable_cameras else 12
        return gym.spaces.Box(low=-np.inf, high=np.inf, shape=(shape_dim,))
        
    def step(self, actions):
        obs, reward, terminated, truncated, info = self._env.step(actions)
        return self._flatten_obs(obs), reward, terminated, truncated, info
        
    def reset(self):
        obs, info = self._env.reset()
        return self._flatten_obs(obs), info
        
    def _flatten_obs(self, obs):
        vec = obs["telemetry"]
        if "tiled_camera" in obs:
            img = obs["tiled_camera"].reshape(vec.shape[0], -1)
            return torch.cat([vec, img], dim=1)
        return vec
        
    @property
    def state_space(self):
        return self._env.state_space
        
    @property
    def action_space(self):
        return self._env.action_space
        
    def state(self):
        return self._env.state()
        
    def render(self, *args, **kwargs):
        return self._env.render(*args, **kwargs)
        
    def close(self):
        self._env.close()

import traceback

try:
    print("Configuring environment...")
    sys.stdout.flush()
    env_cfg = ARCProEnvCfg()
    env_cfg.scene.num_envs = args_cli.num_envs
    env_cfg.enable_cameras = True
    env_cfg.__post_init__()

    print("Instantiating ManagerBasedRLEnv...")
    sys.stdout.flush()
    env = ManagerBasedRLEnv(cfg=env_cfg, render_mode="human")

    print("Wrapping environment...")
    sys.stdout.flush()
    env = IsaacLabWrapper(env)
    env = SKRLFlattenWrapper(env)

    print("Setting up constant action...")
    sys.stdout.flush()

    print("Resetting environment...")
    obs, info = env.reset()

    print("Entering simulation loop...")
    step_count = 0
    while simulation_app.is_running():
        # Action space is 3-dimensional: [steer, throttle, brake]
        action = torch.zeros((args_cli.num_envs, 3), device=env.device)
        
        # Wait 10 steps for the car to drop to the ground before driving
        if step_count > 10:
            action[:, 0] = 0.0  # Steer
            action[:, 1] = 1.0  # Throttle
            action[:, 2] = 0.0  # Brake
            
        obs, reward, terminated, truncated, info = env.step(action)
        step_count += 1

    print("Simulation loop ended.")
    env.close()

except Exception as e:
    print("="*50)
    print("ERROR CAUGHT IN PLAY_SKRL.PY:")
    traceback.print_exc()
    print("="*50)
    sys.stdout.flush()

simulation_app.close()
print("Clean shutdown complete.")
