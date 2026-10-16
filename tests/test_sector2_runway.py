#!/usr/bin/env python3
"""
Verify Sector 2 (Corner 1 Runway Approach) with updated turn tokens and agent_3337500.pt.
Ensures car drives centered down the straight runway without premature left steering.
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


def force_sector_2():
    # Force Sector 2: t1=0.0, t2=1.0, t3=1.0, t4=1.0
    th = (0.0, 1.0, 1.0, 1.0)
    curr_cfg.get_sector_thresholds = lambda: th
    mdp_events.get_sector_thresholds = lambda: th


def main():
    force_sector_2()

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

    ckpt_path = os.path.join(
        WORKSPACE_DIR,
        "logs/ppo_skrl_rollback_clean-baseline/20260930-233029/26-09-30_23-30-29-972175_TelemetryPPO/checkpoints/agent_3337500.pt",
    )
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

    print("\n--- Running Verification of agent_3337500.pt on Sector 2 Runway ---")
    obs, info = env.reset()
    max_steps = 150
    step_count = 0
    done = False

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
        base_env = env.unwrapped
        robot_pos = base_env.scene["robot"].data.root_pos_w[0].cpu().numpy()
        turn_tok = float(obs[0, 0].item())

        if step_count in (10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 120, 150):
            print(
                f"  Step {step_count:3d} | Pos: ({robot_pos[0]:.2f}, {robot_pos[1]:.2f}) | TurnTok: {turn_tok:+.1f} | Spd: {spd:.2f}m/s | Steer: {steer:+.2f} | W: {dist_w:4.1f}cm | Y: {dist_y:4.1f}cm"
            )

        if bool((terminated | truncated)[0].item()):
            done = True
            print(
                f"RESET @ Step {step_count} (Pos: {robot_pos[0]:.2f}, {robot_pos[1]:.2f})"
            )

    if not done:
        print(f"SUCCESS: Completed {max_steps} steps without collision or reset!")

    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
    os._exit(0)
