import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Play SKRL ARCPro Policy")
parser.add_argument("--checkpoint", type=str, default="logs/ppo_skrl/20260701-192503/26-07-01_19-25-03-529362_PPO/checkpoints/best_agent.pt", help="Path to SKRL checkpoint .pt file")
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

from agents.skrl_wrappers import SKRLFlattenWrapper

import traceback

try:
    print("Configuring environment...")
    sys.stdout.flush()
    env_cfg = ARCProEnvCfg()
    env_cfg.scene.num_envs = args_cli.num_envs
    env_cfg.enable_cameras = args_cli.enable_cameras
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
    print(f"{'Step':>6} | {'Speed(m/s)':>10} | {'Reward':>8} | {'MaxSpd':>8} | {'Done'}")
    print("-" * 55)
    sys.stdout.flush()

    step_count = 0
    max_speed = 0.0
    MAX_STEPS = 500  # auto-stop after 500 steps for a clean physics test

    while simulation_app.is_running() and step_count < MAX_STEPS:
        # Action space is 3-dimensional: [steer, throttle, brake]
        action = torch.zeros((args_cli.num_envs, 3), device=env.device)

        # Wait 10 steps for the car to drop to the ground before driving
        if step_count > 10:
            action[:, 0] = 0.0  # Steer
            action[:, 1] = 1.0  # Throttle
            action[:, 2] = 0.0  # Brake

        obs, reward, terminated, truncated, info = env.step(action)
        step_count += 1

        # Speed is index 3 in the telemetry obs vector (fwd velocity m/s)
        speed = float(obs[0, 3].item()) if obs.shape[-1] > 3 else float(obs[0, 0].item())
        rew_val = float(reward[0].item())
        done = bool((terminated | truncated)[0].item())
        max_speed = max(max_speed, abs(speed))

        if step_count % 50 == 0 or done:
            print(f"{step_count:>6} | {speed:>10.3f} | {rew_val:>8.2f} | {max_speed:>8.3f} | {'YES' if done else 'no'}")
            sys.stdout.flush()

        if done:
            print(f"[RESET] Episode ended at step {step_count} — resetting.")
            obs, info = env.reset()
            step_count = 0
            max_speed = 0.0

    print("=" * 55)
    print(f"PHYSICS SUMMARY: max_speed_seen = {max_speed:.3f} m/s over {step_count} steps")
    print("=" * 55)
    sys.stdout.flush()
    env.close()

except Exception as e:
    print("="*50)
    print("ERROR CAUGHT IN PLAY_SKRL.PY:")
    traceback.print_exc()
    print("="*50)
    sys.stdout.flush()

simulation_app.close()
print("Clean shutdown complete.")
