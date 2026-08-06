import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Play SKRL ARCPro Policy")
parser.add_argument("--checkpoint", type=str, default=None,
                    help="Path to SKRL checkpoint .pt file. Auto-detects latest best_agent.pt if omitted.")
parser.add_argument("--num_envs", type=int, default=1, help="Number of parallel environments")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import sys
import os

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, "..", ".."))
if ROOT_DIR not in sys.path:
    sys.path.insert(0, ROOT_DIR)

# Auto-detect latest checkpoint when --checkpoint is not provided
if args_cli.checkpoint is None:
    log_dir = os.path.join(ROOT_DIR, "logs", "ppo_skrl")
    candidates = []
    for root, dirs, files in os.walk(log_dir):
        for f in files:
            if f == "best_agent.pt":
                candidates.append(os.path.join(root, f))
    if not candidates:
        # Fallback: any numbered agent checkpoint
        for root, dirs, files in os.walk(log_dir):
            for f in files:
                if f.endswith(".pt"):
                    candidates.append(os.path.join(root, f))
    if candidates:
        # Pick the most recently modified checkpoint
        args_cli.checkpoint = max(candidates, key=os.path.getmtime)
        print(f"[debug.sh] Auto-detected checkpoint: {args_cli.checkpoint}")
    else:
        print("[debug.sh] WARNING: No checkpoint found. Running with random weights.")

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
from skrl.envs.wrappers.torch import IsaacLabWrapper
import numpy as np
import gymnasium as gym

from agents.skrl_wrappers import SKRLFlattenWrapper

import traceback

class WarmupActionWrapper(gym.Wrapper):
    def __init__(self, env, warmup_steps=10):
        super().__init__(env)
        self.warmup_steps = warmup_steps
        
    def step(self, action):
        if hasattr(self.env.unwrapped, "episode_length_buf"):
            mask = self.env.unwrapped.episode_length_buf < self.warmup_steps
            action = action.clone()
            action[mask, 0] = 0.0   # Steer: 0.0 is center
            action[mask, 1] = -1.0  # Throttle: action * (-20) - 20 = 0.0 (Stop)
        return self.env.step(action)

try:
    print("Configuring environment...")
    sys.stdout.flush()
    env_cfg = ARCProEnvCfg()
    env_cfg.scene.num_envs = args_cli.num_envs
    if args_cli.enable_cameras:
        env_cfg.enable_cameras = True
    env_cfg.__post_init__()

    print("Instantiating ManagerBasedRLEnv...")
    sys.stdout.flush()
    env = ManagerBasedRLEnv(cfg=env_cfg, render_mode="human")
    env = WarmupActionWrapper(env, warmup_steps=10)

    print("Wrapping environment...")
    sys.stdout.flush()
    env = IsaacLabWrapper(env)
    env = SKRLFlattenWrapper(env)

    print("Setting up models...")
    sys.stdout.flush()
    state_space = getattr(env, "state_space", env.observation_space)
    models = {}
    models["policy"] = ARCProActor(env.observation_space, env.action_space, "cuda:0")
    models["value"] = ARCProCritic(state_space, env.action_space, "cuda:0")

    agent_cfg = PPO_DEFAULT_CONFIG.copy()
    agent_cfg["experiment"]["write_interval"] = 0
    agent_cfg["experiment"]["checkpoint_interval"] = 0

    agent = PPO(models=models,
                memory=None,
                cfg=agent_cfg,
                observation_space=env.observation_space,
                action_space=env.action_space,
                device="cuda:0")

    print(f"Loading checkpoint: {args_cli.checkpoint}")
    if args_cli.checkpoint is not None and os.path.exists(args_cli.checkpoint):
        agent.load(args_cli.checkpoint)
        print("Checkpoint loaded successfully.")
    elif args_cli.checkpoint is not None:
        print(f"ERROR: Checkpoint not found: {args_cli.checkpoint}")
        print("Proceeding with random weights for visual debug.")
    else:
        print("No checkpoint. Proceeding with random weights for visual debug.")

    agent.set_mode("eval")

    print("Resetting environment...")
    obs, info = env.reset()

    print("Entering simulation loop...")
    while simulation_app.is_running():
        with torch.no_grad():
            # SKRL policy model forward pass
            # The agent.act method handles everything natively
            action = agent.act(obs, timestep=0, timesteps=0)[0]
        
        obs, reward, terminated, truncated, info = env.step(action)

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
