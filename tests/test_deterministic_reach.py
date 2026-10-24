#!/usr/bin/env python3
"""
Test Deterministic Reach & Survival:
Evaluates the latest trained checkpoint in strict deterministic mode (mean actions,
eval mode, zero exploration noise) starting from the Start Line (Sector 1)
to measure true physical reach, survival steps, and lane centering compared to
the episode lengths reported during training.
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

import arcproLab.mdp.curriculum_config as curr_cfg
import arcproLab.mdp.events as mdp_events
from arcproLab.agents.skrl_models import ARCProActor, ARCProCritic
from arcproLab.arcpro_env_cfg import ARCProEnvCfg


class WarmupActionWrapper(gym.Wrapper):
    """Brakes during initial simulation warmup."""

    def __init__(self, env, warmup_steps=10):
        super().__init__(env)
        self.warmup_steps = warmup_steps

    def step(self, action):
        if hasattr(self.env.unwrapped, "episode_length_buf"):
            mask = self.env.unwrapped.episode_length_buf < self.warmup_steps
            action = action.clone()
            action[mask, 0] = 0.0  # Steer centered
            action[mask, 1] = -1.0  # Brake
        return self.env.step(action)


def set_spawn_sector(sector: int = 1):
    """Force deterministic spawn sector (1 = Start Line, 3 = Intersection Runway)."""
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


def load_policy_weights(agent: PPO, checkpoint_path: str):
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


def get_zone_name(x: float, y: float) -> str:
    """Identifies the approximate circuit zone from (X, Y) coordinates."""
    if x < -14.0:
        if y > 2.0:
            return "West Straight (Start Line)"
        elif y > -4.5:
            return "Corner 1 Approach Runway"
        else:
            return "Corner 1 (Hairpin Left)"
    elif y < -5.5:
        if x < -7.0:
            return "Leg 2 Straightaway"
        else:
            return "Corner 2 / Intersection Approach"
    elif x > -5.0 and y < -2.0:
        return "Intersection Left Turn"
    elif x > -5.0 and y >= -2.0 and y < 4.0:
        return "North Straight (Sweeping Bend)"
    elif y >= 4.0:
        if x > -10.0:
            return "Top North Straight"
        else:
            return "Corner 4 Approach / Hairpin"
    return "Circuit Track"


def run_reach_evaluation(env, agent, ckpt_path: str, max_steps: int = 2500):
    ckpt_name = os.path.basename(ckpt_path)
    print("\n" + "=" * 90)
    print(f"  DETERMINISTIC EVALUATION: {ckpt_name}")
    print(
        f"  Max Steps: {max_steps} | Mode: EVAL (Mean Actions) | Spawn: Sector 1 (Start Line)"
    )
    print("=" * 90)

    load_policy_weights(agent, ckpt_path)
    set_spawn_sector(sector=1)

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

    header = f"{'Step':<6} | {'Zone':<28} | {'Pos (X, Y)':<16} | {'Speed':<8} | {'Steer':<7} | {'White':<7} | {'Yellow':<7} | {'LatErr':<8} | {'WPs'}"
    print(header)
    print("-" * 110)

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
        wps_val = (
            float(info["track_wp_delta"].float().mean().item())
            if "track_wp_delta" in info
            else 0.0
        )
        lat_err = float(info["lat_err"][0].item()) * 100.0 if "lat_err" in info else 0.0
        rew_val = float(reward.float().mean().item())

        total_rew += rew_val
        speeds.append(speed_val)
        steers.append(steer_val)
        min_dist_w = min(min_dist_w, dist_w)
        min_dist_y = min(min_dist_y, dist_y)
        max_wps = max(max_wps, wps_val)

        base_env = env.unwrapped
        pos = base_env.scene["robot"].data.root_pos_w[0].cpu().numpy()
        origin = base_env.scene.env_origins[0].cpu().numpy()
        local_x = float(pos[0] - origin[0])
        local_y = float(pos[1] - origin[1])
        zone = get_zone_name(local_x, local_y)

        # Log periodically or at key milestones
        if (
            step_count
            in (
                1,
                10,
                25,
                50,
                100,
                150,
                200,
                250,
                300,
                400,
                500,
                600,
                750,
                900,
                1000,
                1200,
                1400,
                1600,
                1800,
                2000,
                2200,
                2400,
            )
            or step_count % 100 == 0
        ):
            print(
                f"{step_count:<6d} | {zone:<28} | ({local_x:6.2f}, {local_y:6.2f}) | {speed_val:5.2f}m/s | {steer_val:+5.2f} | {dist_w:5.1f}cm | {dist_y:5.1f}cm | {lat_err:+6.1f}cm | {wps_val:4.0f}"
            )

        is_term = bool(terminated[0].item())
        is_trunc = bool(truncated[0].item())
        if is_term or is_trunc:
            done = True
            final_pos = (local_x, local_y)
            term_reason = "RESET"
            if dist_w < 4.0:
                term_reason = f"OUTER_WHITE_HIT (W={dist_w:.1f}cm)"
            elif dist_y < 2.0:
                term_reason = f"INNER_YELLOW_HIT (Y={dist_y:.1f}cm)"
            outcome = f"{term_reason} @ Step {step_count} in {zone} at Pos ({final_pos[0]:.2f}, {final_pos[1]:.2f})"

    if not done:
        base_env = env.unwrapped
        pos = base_env.scene["robot"].data.root_pos_w[0].cpu().numpy()
        origin = base_env.scene.env_origins[0].cpu().numpy()
        final_pos = (float(pos[0] - origin[0]), float(pos[1] - origin[1]))
        outcome = f"SUCCESSFULLY TRAVERSED ALL {max_steps} STEPS! Final Pos ({final_pos[0]:.2f}, {final_pos[1]:.2f})"

    print("-" * 110)
    print("\n" + "=" * 80)
    print("  DETERMINISTIC REACH EVALUATION SUMMARY")
    print("=" * 80)
    print(f"  Checkpoint Tested : {ckpt_name}")
    print(f"  Outcome           : {outcome}")
    print(f"  Total Steps       : {step_count} / {max_steps}")
    print(
        f"  Avg Speed         : {np.mean(speeds):.2f} m/s (Max: {np.max(speeds):.2f} m/s)"
    )
    print(f"  Min White Margin  : {min_dist_w:.1f} cm")
    print(f"  Min Yellow Margin : {min_dist_y:.1f} cm")
    print(f"  Cumulative WPs    : {max_wps:.0f} waypoints")
    print("=" * 80 + "\n")

    return {
        "steps": step_count,
        "outcome": outcome,
        "avg_speed": float(np.mean(speeds)),
        "min_dist_w": float(min_dist_w),
        "min_dist_y": float(min_dist_y),
        "max_wps": float(max_wps),
        "final_pos": final_pos,
    }


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
    if not ckpt_path:
        print("ERROR: No checkpoint found!")
        env.close()
        return

    run_reach_evaluation(env, agent, ckpt_path, max_steps=2500)
    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
    os._exit(0)
