#!/usr/bin/env python3
"""
Headless Checkpoint Benchmark Evaluator:
Evaluates robot performance across trained checkpoints in Sector 4 (Intersection)
and reports detailed telemetry: survival steps, speed profile, steering lock,
line clearances, waypoints traversed, and outcome.
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
            action[mask, 0] = 0.0  # Steer: 0.0 is center
            action[mask, 1] = -1.0  # Throttle: action * (-20) - 20 = 0.0 (Stop)
        return self.env.step(action)


def set_spawn_sector(sector: int = 4):
    """Force deterministic spawn sector (4 = Intersection)."""
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
    """Load policy weights safely into agent."""
    if not os.path.exists(checkpoint_path):
        raise FileNotFoundError(f"Checkpoint not found: {checkpoint_path}")

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


def evaluate_checkpoint(
    env,
    agent,
    ckpt_name: str,
    ckpt_path: str,
    num_episodes: int = 2,
    max_steps: int = 350,
):
    print(f"\n{'=' * 75}")
    print(f"EVALUATING: {ckpt_name}")
    print(f"Path: {ckpt_path}")
    print(f"{'=' * 75}")

    load_policy_weights(agent, ckpt_path)

    results = []

    for ep in range(num_episodes):
        obs, info = env.reset()

        step_count = 0
        total_rew = 0.0
        speeds = []
        steers = []
        min_dist_w = float("inf")
        min_dist_y = float("inf")
        max_wps = 0
        done = False
        outcome = "COMPLETED"
        final_pos = (0.0, 0.0)

        while step_count < max_steps and not done:
            with torch.no_grad():
                _, _, outputs = agent.act(obs, timestep=0, timesteps=0)
                action = outputs.get("mean_actions", None)
                if action is None:
                    action = agent.policy.compute({"states": obs})[0]
                action = action.clone()

            obs, reward, terminated, truncated, info = env.step(action)
            step_count += 1

            # Telemetry extraction
            steer_val = float(action[0, 0].item())
            speed_val = (
                float(info["speed_mps"].float().mean().item())
                if "speed_mps" in info
                else 0.0
            )
            dist_w = (
                float(info["dist_white"][0].item()) * 100.0
                if "dist_white" in info
                else 0.0
            )
            dist_y = (
                float(info["dist_yellow"][0].item()) * 100.0
                if "dist_yellow" in info
                else 0.0
            )
            wps_val = (
                float(info["track_wp_delta"].float().mean().item())
                if "track_wp_delta" in info
                else 0.0
            )
            rew_val = float(reward.float().mean().item())

            total_rew += rew_val
            speeds.append(speed_val)
            steers.append(steer_val)
            min_dist_w = min(min_dist_w, dist_w)
            min_dist_y = min(min_dist_y, dist_y)
            max_wps = max(max_wps, wps_val)

            # Periodic logging
            if step_count % 50 == 0 or step_count in (10, 20, 100, 150, 190):
                base_env = env.unwrapped
                robot_pos = base_env.scene["robot"].data.root_pos_w[0].cpu().numpy()
                lat_err_cm = (
                    float(info["lat_err"][0].item()) * 100.0
                    if "lat_err" in info
                    else 0.0
                )
                print(
                    f"  [Ep {ep + 1} | Step {step_count:3d}] Spd: {speed_val:.2f}m/s | Steer: {steer_val:+.2f} | Pos: ({robot_pos[0]:.2f}, {robot_pos[1]:.2f}) | LatErr: {lat_err_cm:+.1f}cm | W: {dist_w:4.1f}cm | Y: {dist_y:4.1f}cm | WPs: {wps_val:.0f}"
                )

            is_term = bool(terminated[0].item())
            is_trunc = bool(truncated[0].item())
            if is_term or is_trunc:
                done = True
                base_env = env.unwrapped
                robot_pos = base_env.scene["robot"].data.root_pos_w[0].cpu().numpy()
                final_pos = (robot_pos[0], robot_pos[1])
                outcome = f"RESET @ Step {step_count} (Pos: {final_pos[0]:.2f}, {final_pos[1]:.2f})"

        if not done:
            base_env = env.unwrapped
            robot_pos = base_env.scene["robot"].data.root_pos_w[0].cpu().numpy()
            final_pos = (robot_pos[0], robot_pos[1])
            outcome = f"COMPLETED {max_steps} steps (Pos: {final_pos[0]:.2f}, {final_pos[1]:.2f})"

        avg_speed = np.mean(speeds) if speeds else 0.0
        max_speed = np.max(speeds) if speeds else 0.0
        mean_steer = np.mean(steers) if steers else 0.0
        max_steer = np.max(steers) if steers else 0.0

        ep_summary = {
            "episode": ep + 1,
            "steps": step_count,
            "outcome": outcome,
            "avg_speed": avg_speed,
            "max_speed": max_speed,
            "mean_steer": mean_steer,
            "max_steer": max_steer,
            "min_dist_w_cm": min_dist_w,
            "min_dist_y_cm": min_dist_y,
            "max_wps": max_wps,
            "total_rew": total_rew,
        }
        results.append(ep_summary)
        print(
            f"  --> Ep {ep + 1} Result: {outcome} | Steps: {step_count} | AvgSpd: {avg_speed:.2f}m/s | MaxSteer: {max_steer:+.2f} | MinW: {min_dist_w:.1f}cm | MinY: {min_dist_y:.1f}cm | WPs: {max_wps:.0f} | Rew: {total_rew:.1f}\n"
        )

    return results


def main():
    print("=" * 75)
    print("  ARCPro RL - Headless Multi-Checkpoint Benchmark (Sector 4: Intersection)")
    print("=" * 75)

    set_spawn_sector(sector=4)

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

    checkpoints_to_test = [
        (
            "Checkpoint 1: agent_3337500.pt (Pre-Fix Baseline)",
            os.path.join(
                WORKSPACE_DIR,
                "logs/ppo_skrl_rollback_clean-baseline/20260930-233029/26-09-30_23-30-29-972175_TelemetryPPO/checkpoints/agent_3337500.pt",
            ),
        ),
        (
            "Checkpoint 2: agent_3387500.pt (+50k Pre-Braking Steps)",
            os.path.join(
                WORKSPACE_DIR,
                "logs/ppo_skrl_rollback_clean-baseline/20261001-212131/26-10-01_21-21-31-424691_TelemetryPPO/checkpoints/agent_3387500.pt",
            ),
        ),
        (
            "Checkpoint 3: agent_3437500.pt (+100k Pre-Braking Steps, Latest)",
            os.path.join(
                WORKSPACE_DIR,
                "logs/ppo_skrl_rollback_clean-baseline/20261001-212131/26-10-01_21-21-31-424691_TelemetryPPO/checkpoints/agent_3437500.pt",
            ),
        ),
    ]

    all_summaries = {}
    for name, path in checkpoints_to_test:
        if os.path.exists(path):
            all_summaries[name] = evaluate_checkpoint(
                env, agent, name, path, num_episodes=2, max_steps=350
            )
        else:
            print(f"Skipping missing checkpoint: {path}")

    # Final Comparison Table
    print("\n" + "=" * 85)
    print("FINAL HEADLESS CHECKPOINT PERFORMANCE COMPARISON TABLE")
    print("=" * 85)
    header = f"{'Checkpoint':<42} | {'Steps':<6} | {'AvgSpd':<8} | {'MaxSteer':<8} | {'MinW':<7} | {'MinY':<7} | {'WPs':<5} | {'Outcome'}"
    print(header)
    print("-" * 85)
    for name, ep_list in all_summaries.items():
        for res in ep_list:
            outcome_str = res["outcome"][:22]
            print(
                f"{name[:42]:<42} | {res['steps']:<6d} | {res['avg_speed']:<6.2f}m/s | {res['max_steer']:<+7.2f} | {res['min_dist_w_cm']:<5.1f}cm | {res['min_dist_y_cm']:<5.1f}cm | {res['max_wps']:<5.0f} | {outcome_str}"
            )
    print("=" * 85)

    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
    os._exit(0)
