#!/usr/bin/env python3
"""
Diagnostic script to isolate:
1. Sector 4: What happens after the intersection on the curve? (Does it cut across the road?)
2. Sector 2: What happens when driving through Corner 1 into the intersection? (Where does it crash?)
"""

import os
import sys

import numpy as np

WORKSPACE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if WORKSPACE_DIR not in sys.path:
    sys.path.insert(0, WORKSPACE_DIR)
ARCPRO_LAB_DIR = os.path.join(WORKSPACE_DIR, "arcproLab")
if ARCPRO_LAB_DIR not in sys.path:
    sys.path.insert(0, ARCPRO_LAB_DIR)

from isaaclab.app import AppLauncher

launcher_args = {"headless": True, "num_envs": 1, "enable_cameras": True}
app_launcher = AppLauncher(launcher_args)
simulation_app = app_launcher.app

import gymnasium as gym
import torch
from agents.skrl_wrappers import SKRLFlattenWrapper
from isaaclab.envs import ManagerBasedRLEnv
from skrl.agents.torch.ppo import PPO, PPO_DEFAULT_CONFIG
from skrl.envs.wrappers.torch import IsaacLabWrapper

from arcproLab.agents.skrl_models import ARCProActor, ARCProCritic
from arcproLab.arcpro_env_cfg import ARCProEnvCfg


class WarmupActionWrapper(gym.Wrapper):
    def __init__(self, env, warmup_steps=10):
        super().__init__(env)
        self.warmup_steps = warmup_steps

    def step(self, action):
        if hasattr(self.env.unwrapped, "episode_length_buf"):
            mask = self.env.unwrapped.episode_length_buf < self.warmup_steps
            action = action.clone()
            action[mask, 0] = 0.0
            action[mask, 1] = -1.0
        return self.env.step(action)


def find_latest_checkpoint() -> str:
    candidate_dirs = [
        os.path.join(WORKSPACE_DIR, "logs/ppo_skrl_rollback_clean-baseline"),
        os.path.join(WORKSPACE_DIR, "logs/ppo_skrl"),
    ]
    ckpts = []
    for cdir in candidate_dirs:
        if not os.path.isdir(cdir):
            continue
        for root, _, files in os.walk(cdir):
            if "corrupted" in root:
                continue
            for f in files:
                if f.startswith("agent_") and f.endswith(".pt"):
                    p = os.path.join(root, f)
                    ckpts.append((os.path.getmtime(p), p))
    if ckpts:
        ckpts.sort()
        return ckpts[-1][1]
    return ""


def run_sector_diagnostic(
    env, agent, ckpt_path: str, sector: int, max_steps: int = 600
):
    os.environ["ARCPRO_FORCE_SECTOR"] = str(sector)
    ckpt_name = os.path.basename(ckpt_path)
    print("\n" + "=" * 105)
    print(
        f"  DIAGNOSTIC RUN: SECTOR {sector} | CHECKPOINT: {ckpt_name} | MAX STEPS: {max_steps}"
    )
    print("=" * 105)

    obs, info = env.reset()

    step_count = 0
    done = False
    outcome = "COMPLETED"

    header = (
        f"{'Step':<5} | {'Pos (X, Y)':<16} | {'Speed':<7} | {'Steer':<6} | "
        f"{'White':<6} | {'Yellow':<6} | {'LatErr':<7} | {'HeadErr':<8} | {'Token':<5} | {'WPs'}"
    )
    print(header)
    print("-" * 105)

    prev_pos = None

    while step_count < max_steps and not done:
        with torch.no_grad():
            _, _, outputs = agent.act(obs, timestep=0, timesteps=0)
            action = outputs.get("mean_actions", None)
            if action is None:
                action = agent.policy.compute({"states": obs})[0]
            action = action.clone()

        obs, reward, terminated, truncated, info = env.step(action)
        step_count += 1

        steer_val = float(action[0, 0].item())
        speed_val = (
            float(info["speed_mps"].float().mean().item())
            if "speed_mps" in info
            else 0.0
        )
        dist_w = (
            float(info["dist_white"][0].item()) * 100.0 if "dist_white" in info else 0.0
        )
        dist_y = (
            float(info["dist_yellow"][0].item()) * 100.0
            if "dist_yellow" in info
            else 0.0
        )
        lat_err = float(info["lat_err"][0].item()) * 100.0 if "lat_err" in info else 0.0
        head_err = (
            float(info["head_err"][0].item()) * 180.0 / np.pi
            if "head_err" in info
            else 0.0
        )
        wps_val = (
            float(info["track_wp_delta"].float().mean().item())
            if "track_wp_delta" in info
            else 0.0
        )

        token_val = 0.0
        if hasattr(env.unwrapped, "extras") and "turn_token" in env.unwrapped.extras:
            token_val = float(env.unwrapped.extras["turn_token"][0].item())

        base_env = env.unwrapped
        pos = base_env.scene["robot"].data.root_pos_w[0].cpu().numpy()
        origin = base_env.scene.env_origins[0].cpu().numpy()
        local_x = float(pos[0] - origin[0])
        local_y = float(pos[1] - origin[1])

        # Log every 25 steps or when significant change happens
        if (
            step_count
            in (
                1,
                10,
                25,
                50,
                75,
                100,
                125,
                150,
                175,
                200,
                225,
                250,
                275,
                300,
                350,
                400,
                450,
                500,
            )
            or step_count % 50 == 0
        ):
            print(
                f"{step_count:<5d} | ({local_x:6.2f}, {local_y:6.2f}) | {speed_val:5.2f}m/s | {steer_val:+5.2f} | "
                f"{dist_w:5.1f}cm | {dist_y:5.1f}cm | {lat_err:+6.1f}cm | {head_err:+6.1f}° | {token_val:+4.1f} | {wps_val:4.0f}"
            )

        is_term = bool(terminated[0].item())
        is_trunc = bool(truncated[0].item())
        if is_term or is_trunc:
            done = True
            pos_before_reset = prev_pos if prev_pos is not None else (local_x, local_y)
            outcome = (
                f"RESET @ Step {step_count} at Pos ({pos_before_reset[0]:.2f}, {pos_before_reset[1]:.2f}) "
                f"| Spd: {speed_val:.2f}m/s, Steer: {steer_val:+.2f}, White: {dist_w:.1f}cm, Yellow: {dist_y:.1f}cm, LatErr: {lat_err:+.1f}cm"
            )
            print("-" * 105)
            print(f"  >>> TERMINATION TRIGGERED: {outcome}")

        prev_pos = (local_x, local_y)

    if not done:
        outcome = f"SUCCESSFULLY COMPLETED ALL {max_steps} STEPS! Final Pos ({prev_pos[0]:.2f}, {prev_pos[1]:.2f})"
        print("-" * 105)
        print(f"  >>> {outcome}")

    print("=" * 105 + "\n")
    return outcome


def main():
    env_cfg = ARCProEnvCfg()
    env_cfg.scene.num_envs = 1
    env_cfg.enable_cameras = True
    env_cfg.__post_init__()

    env = ManagerBasedRLEnv(cfg=env_cfg)
    env = WarmupActionWrapper(env, warmup_steps=10)
    env = IsaacLabWrapper(env)
    env = SKRLFlattenWrapper(env)

    state_space = getattr(env, "state_space", env.observation_space)
    models = {
        "policy": ARCProActor(env.observation_space, env.action_space, "cuda:0"),
        "value": ARCProCritic(state_space, env.action_space, "cuda:0"),
    }
    agent_cfg = PPO_DEFAULT_CONFIG.copy()
    agent_cfg["experiment"]["write_interval"] = 0
    agent_cfg["experiment"]["checkpoint_interval"] = 0

    agent = PPO(
        models=models,
        memory=None,
        cfg=agent_cfg,
        observation_space=env.observation_space,
        action_space=env.action_space,
        device="cuda:0",
    )

    ckpt_path = find_latest_checkpoint()
    print(f"Loading Checkpoint: {ckpt_path}")
    ckpt_data = torch.load(ckpt_path, map_location=agent.device)
    if isinstance(ckpt_data, dict):
        if "policy" in ckpt_data:
            agent.policy.load_state_dict(ckpt_data["policy"])
        elif "models" in ckpt_data and "policy" in ckpt_data["models"]:
            agent.policy.load_state_dict(ckpt_data["models"]["policy"])
        else:
            agent.policy.load_state_dict(ckpt_data)
    else:
        agent.policy.load_state_dict(ckpt_data)
    agent.set_mode("eval")

    # Run Sector 4 diagnostic
    run_sector_diagnostic(env, agent, ckpt_path, sector=4, max_steps=500)

    # Run Sector 2 diagnostic
    run_sector_diagnostic(env, agent, ckpt_path, sector=2, max_steps=500)

    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
    os._exit(0)
