#!/usr/bin/env python3
"""
Test checkpoints on Corner 1 (Sector 2):
Tests how different candidate checkpoints handle Corner 1 (runway approach & turn).
Reports steps survived, speed, steer, and line clearances.
"""

import sys
import os

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

import torch
import numpy as np
import gymnasium as gym

from isaaclab.envs import ManagerBasedRLEnv
from arcproLab.arcpro_env_cfg import ARCProEnvCfg
from arcproLab.agents.skrl_models import ARCProActor, ARCProCritic
from agents.skrl_wrappers import SKRLFlattenWrapper
from skrl.envs.wrappers.torch import IsaacLabWrapper
from skrl.agents.torch.ppo import PPO, PPO_DEFAULT_CONFIG
import arcproLab.mdp.curriculum_config as curr_cfg
import arcproLab.mdp.events as mdp_events


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


def set_spawn_sector(sector: int = 2):
    sector_thresholds = {
        1: (1.0, 1.0, 1.0, 1.0),
        2: (0.0, 1.0, 1.0, 1.0),
        3: (0.0, 0.0, 1.0, 1.0),
        4: (0.0, 0.0, 0.0, 1.0),
        5: (0.0, 0.0, 0.0, 0.0),
    }
    th = sector_thresholds[sector]
    sector_weights = [0.0, 0.0, 0.0, 0.0, 0.0]
    sector_weights[sector - 1] = 1.0
    curr_cfg._CACHED_CONFIG["sector_weights"] = sector_weights
    curr_cfg._CACHED_THRESHOLDS = list(th)
    curr_cfg.get_sector_thresholds = lambda: th
    mdp_events.get_sector_thresholds = lambda: th


def load_policy_weights(agent: PPO, checkpoint_path: str):
    ckpt_data = torch.load(checkpoint_path, map_location=agent.device)
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


def test_checkpoint_on_c1(env, agent, name: str, path: str, max_steps: int = 350):
    if not os.path.exists(path):
        print(f"Skipping {name}: file not found ({path})")
        return

    print(f"\n--- Testing {name} ---")
    load_policy_weights(agent, path)

    obs, info = env.reset()
    step_count = 0
    speeds = []
    steers = []
    min_w = float("inf")
    min_y = float("inf")
    done = False
    outcome = "COMPLETED"

    while step_count < max_steps and not done:
        with torch.no_grad():
            _, _, outputs = agent.act(obs, timestep=0, timesteps=0)
            action = outputs.get("mean_actions", None)
            if action is None:
                action = agent.policy.compute({"states": obs})[0]
            action = action.clone()

        obs, reward, terminated, truncated, info = env.step(action)
        step_count += 1

        spd = (
            float(info["speed_mps"].float().mean().item())
            if "speed_mps" in info
            else 0.0
        )
        steer = float(action[0, 0].item())
        dist_w = (
            float(info["dist_white"][0].item()) * 100.0 if "dist_white" in info else 0.0
        )
        dist_y = (
            float(info["dist_yellow"][0].item()) * 100.0
            if "dist_yellow" in info
            else 0.0
        )
        wps = (
            float(info["track_wp_delta"].float().mean().item())
            if "track_wp_delta" in info
            else 0.0
        )

        speeds.append(spd)
        steers.append(steer)
        min_w = min(min_w, dist_w)
        min_y = min(min_y, dist_y)

        if (
            step_count in (10, 30, 50, 80, 100, 120, 150, 200, 300)
            or step_count % 50 == 0
        ):
            base_env = env.unwrapped
            robot_pos = base_env.scene["robot"].data.root_pos_w[0].cpu().numpy()
            print(
                f"  Step {step_count:3d} | Pos: ({robot_pos[0]:.2f}, {robot_pos[1]:.2f}) | Spd: {spd:.2f}m/s | Steer: {steer:+.2f} | W: {dist_w:4.1f}cm | Y: {dist_y:4.1f}cm | WPs: {wps:.0f}"
            )

        if bool((terminated | truncated)[0].item()):
            done = True
            base_env = env.unwrapped
            robot_pos = base_env.scene["robot"].data.root_pos_w[0].cpu().numpy()
            outcome = f"RESET @ Step {step_count} (Pos: {robot_pos[0]:.2f}, {robot_pos[1]:.2f})"

    if not done:
        base_env = env.unwrapped
        robot_pos = base_env.scene["robot"].data.root_pos_w[0].cpu().numpy()
        outcome = (
            f"COMPLETED {max_steps} steps (Pos: {robot_pos[0]:.2f}, {robot_pos[1]:.2f})"
        )

    avg_spd = np.mean(speeds) if speeds else 0.0
    print(
        f"Result {name}: {outcome} | AvgSpd: {avg_spd:.2f}m/s | MinW: {min_w:.1f}cm | MinY: {min_y:.1f}cm | WPs: {wps:.0f}\n"
    )


def main():
    set_spawn_sector(sector=2)  # Sector 2: Corner 1 Runway Approach

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

    ckpts = [
        (
            "Current Latest (agent_3437500.pt)",
            os.path.join(
                WORKSPACE_DIR,
                "logs/ppo_skrl_rollback_clean-baseline/20261001-212131/26-10-01_21-21-31-424691_TelemetryPPO/checkpoints/agent_3437500.pt",
            ),
        ),
        (
            "Pre-Intersection-Fix (agent_3337500.pt)",
            os.path.join(
                WORKSPACE_DIR,
                "logs/ppo_skrl_rollback_clean-baseline/20260930-233029/26-09-30_23-30-29-972175_TelemetryPPO/checkpoints/agent_3337500.pt",
            ),
        ),
        (
            "Landmark Baseline (agent_2587500.pt)",
            os.path.join(
                WORKSPACE_DIR,
                "archive/landmark_checkpoints/agent_2587500_corner_plateau_baseline.pt",
            ),
        ),
        (
            "Landmark Champion (agent_4641500.pt)",
            os.path.join(
                WORKSPACE_DIR,
                "archive/landmark_checkpoints/agent_4641500_realsense_champion.pt",
            ),
        ),
    ]

    for name, path in ckpts:
        test_checkpoint_on_c1(env, agent, name, path)

    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
    os._exit(0)
