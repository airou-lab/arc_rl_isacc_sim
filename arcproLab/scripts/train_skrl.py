import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Train SKRL AAC policy for ARCPro Lane Following.")
parser.add_argument("--num_envs", type=int, default=16, help="Number of parallel simulation environments.")
parser.add_argument("--seed", type=int, default=42, help="Seed for the environment.")
parser.add_argument("--total_timesteps", type=int, default=5000000, help="Total timesteps to train.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import os
import sys
import torch
import warnings
from datetime import datetime

warnings.filterwarnings("ignore", category=DeprecationWarning)

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, "..", ".."))
ARCPRO_LAB_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, ".."))
if ROOT_DIR not in sys.path:
    sys.path.insert(0, ROOT_DIR)
if ARCPRO_LAB_DIR not in sys.path:
    sys.path.insert(0, ARCPRO_LAB_DIR)

from isaaclab.envs import ManagerBasedRLEnv
from arcpro_env_cfg import ARCProEnvCfg

from skrl.envs.wrappers.torch import IsaacLabWrapper, Wrapper
from skrl.agents.torch.ppo import PPO, PPO_DEFAULT_CONFIG
from skrl.trainers.torch import SequentialTrainer
from skrl.memories.torch import RandomMemory
import numpy as np
import gymnasium as gym

class SKRLFlattenWrapper(Wrapper):
    def __init__(self, env):
        super().__init__(env)
        
    @property
    def observation_space(self):
        # 12 telemetry + 150528 vision if camera is enabled
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

# Import the new SKRL models
from agents.skrl_models import ARCProActor, ARCProCritic

def main():
    # 1. Setup Environment Configuration
    env_cfg = ARCProEnvCfg()
    env_cfg.scene.num_envs = args_cli.num_envs
    env_cfg.enable_cameras = args_cli.enable_cameras 
    env_cfg.__post_init__() 

    # 2. Create Environment
    env = ManagerBasedRLEnv(cfg=env_cfg)
    
    # 3. Wrap for SKRL
    # The IsaacLabWrapper automatically extracts the "critic" observation group
    # (if it exists) into env.state_space and env.state(), enabling AAC.
    env = IsaacLabWrapper(env)
    
    # 3.5 Flatten for SKRL RandomMemory limitation
    env = SKRLFlattenWrapper(env)
    
    # 4. Define Memory (reduce size to prevent OOM)
    memory = RandomMemory(memory_size=128, num_envs=env.num_envs, device="cuda:0")
    
    # 5. Define Models
    models = {}
    models["policy"] = ARCProActor(env.observation_space, env.action_space, "cuda:0")
    
    # Use env.state_space for the Critic if available, fallback to observation_space
    state_space = getattr(env, "state_space", env.observation_space)
    models["value"] = ARCProCritic(state_space, env.action_space, "cuda:0")
    
    # 6. Configure Agent
    cfg_ppo = PPO_DEFAULT_CONFIG.copy()
    cfg_ppo["rollouts"] = 128
    cfg_ppo["mini_batches"] = 16  # Small mini-batch size (128*4/16 = 32 images per backprop)
    cfg_ppo["learning_rate"] = 1e-4
    cfg_ppo["random_timesteps"] = 0
    cfg_ppo["learning_starts"] = 0
    cfg_ppo["state_preprocessor"] = None
    cfg_ppo["value_preprocessor"] = None
    log_dir = os.path.join("logs", "ppo_skrl", datetime.now().strftime("%Y%m%d-%H%M%S"))
    cfg_ppo["experiment"]["directory"] = log_dir
    cfg_ppo["experiment"]["write_interval"] = 100
    cfg_ppo["experiment"]["checkpoint_interval"] = max(1, 500000 // env.num_envs)
    
    agent = PPO(models=models, memory=memory, cfg=cfg_ppo, observation_space=env.observation_space, action_space=env.action_space, device="cuda:0")
    
    # 7. Train
    print(f"Starting SKRL AAC training for {args_cli.total_timesteps} steps...")
    trainer = SequentialTrainer(cfg={"timesteps": args_cli.total_timesteps}, env=env, agents=agent)
    trainer.train()

    env.close()

if __name__ == "__main__":
    main()
