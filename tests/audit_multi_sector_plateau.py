#!/usr/bin/env python3
"""
Diagnostic Multi-Sector Checkpoint Audit:
Evaluates candidate checkpoints (3.33M baseline, 3.58M, and 3.63M) across
Sector 1 (Start Line), Sector 2 (Corner 1 Approach), and Sector 4 (Intersection).
Pinpoints exact termination coordinates, line contact mechanisms, and telemetry.
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


def set_spawn_sector(sector: int):
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


def run_single_eval(env, agent, ckpt_name: str, sector: int, max_steps: int):
    set_spawn_sector(sector)
    obs, info = env.reset()

    step_count = 0
    speeds = []
    steers = []
    min_dist_w = float("inf")
    min_dist_y = float("inf")
    max_wps = 0
    done = False
    termination_reason = "SURVIVED"

    recent_history = []

    while step_count < max_steps and not done:
        with torch.no_grad():
            _, _, outputs = agent.act(obs, timestep=0, timesteps=0)
            action = outputs.get("mean_actions", None)
            if action is None:
                action = agent.policy.compute({"states": obs})[0]
            action = action.clone()

        obs, reward, terminated, truncated, info = env.step(action)
        step_count += 1

        base_env = env.unwrapped
        robot_pos = base_env.scene["robot"].data.root_pos_w[0].cpu().numpy()

        steer_val = float(action[0, 0].item())
        speed_val = (
            float(info["speed_mps"].float().mean().item())
            if "speed_mps" in info
            else 0.0
        )
        dist_w_cm = (
            float(info["dist_white"][0].item()) * 100.0 if "dist_white" in info else 0.0
        )
        dist_y_cm = (
            float(info["dist_yellow"][0].item()) * 100.0
            if "dist_yellow" in info
            else 0.0
        )
        wps_val = (
            float(info["track_wp_delta"].float().mean().item())
            if "track_wp_delta" in info
            else 0.0
        )
        lat_err_cm = (
            float(info["lat_err"][0].item()) * 100.0 if "lat_err" in info else 0.0
        )
        head_err_deg = (
            float(info["head_err"][0].item()) * 180.0 / np.pi
            if "head_err" in info
            else 0.0
        )

        speeds.append(speed_val)
        steers.append(steer_val)
        min_dist_w = min(min_dist_w, dist_w_cm)
        min_dist_y = min(min_dist_y, dist_y_cm)
        max_wps = max(max_wps, wps_val)

        step_snapshot = (
            step_count,
            robot_pos[0],
            robot_pos[1],
            speed_val,
            steer_val,
            dist_w_cm,
            dist_y_cm,
            lat_err_cm,
        )
        recent_history.append(step_snapshot)
        if len(recent_history) > 5:
            recent_history.pop(0)

        is_term = bool(terminated[0].item())
        is_trunc = bool(truncated[0].item())

        if is_term or is_trunc:
            done = True
            # Analyze termination reason
            if dist_w_cm < 4.0:
                termination_reason = (
                    f"OUTER WHITE LINE HIT (dist_w={dist_w_cm:.1f}cm < 4.0cm)"
                )
            elif dist_y_cm < 2.0:
                termination_reason = (
                    f"INNER YELLOW LINE HIT (dist_y={dist_y_cm:.1f}cm < 2.0cm)"
                )
            elif abs(lat_err_cm) > 50.0:
                termination_reason = (
                    f"OFF TRACK (|lat_err|={abs(lat_err_cm):.1f}cm > 50cm)"
                )
            elif is_trunc:
                termination_reason = "TRUNCATED (Timeout)"
            else:
                termination_reason = "GROUND CONTACT / OFF-SHOULDER"

    final_pos = base_env.scene["robot"].data.root_pos_w[0].cpu().numpy()
    avg_speed = float(np.mean(speeds)) if speeds else 0.0

    print(f"  [Sector {sector}] {ckpt_name}:")
    print(f"    Outcome: {termination_reason} @ Step {step_count}")
    print(
        f"    Final Pos: ({final_pos[0]:.2f}, {final_pos[1]:.2f}) | AvgSpd: {avg_speed:.2f}m/s"
    )
    print(
        f"    Min White: {min_dist_w:.1f}cm | Min Yellow: {min_dist_y:.1f}cm | Max WPs: {max_wps:.0f}"
    )
    if done and recent_history:
        print("    Recent 3 steps before reset:")
        for s_idx, x, y, spd, st, w, y_dist, lat in recent_history[-3:]:
            print(
                f"      Step {s_idx:3d}: Pos=({x:.2f}, {y:.2f}) | Spd={spd:.2f}m/s | Steer={st:+.2f} | W={w:4.1f}cm | Y={y_dist:4.1f}cm | LatErr={lat:+.1f}cm"
            )
    print()

    return {
        "sector": sector,
        "checkpoint": ckpt_name,
        "steps": step_count,
        "outcome": termination_reason,
        "final_pos": (float(final_pos[0]), float(final_pos[1])),
        "avg_speed": avg_speed,
        "min_dist_w_cm": min_dist_w,
        "min_dist_y_cm": min_dist_y,
        "max_wps": max_wps,
    }


def main():
    print("=" * 80)
    print("  ARCPro RL - Diagnostic Multi-Sector Plateau Audit")
    print("=" * 80)

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

    checkpoints = [
        (
            "agent_3337500.pt (Baseline)",
            os.path.join(
                WORKSPACE_DIR,
                "logs/ppo_skrl_rollback_clean-baseline/20260930-233029/26-09-30_23-30-29-972175_TelemetryPPO/checkpoints/agent_3337500.pt",
            ),
        ),
        (
            "agent_3587500.pt (Pre-Recycle)",
            os.path.join(
                WORKSPACE_DIR,
                "logs/ppo_skrl_rollback_clean-baseline/20261007-111144/26-10-07_11-11-44-080499_TelemetryPPO/checkpoints/agent_3587500.pt",
            ),
        ),
        (
            "agent_3637500.pt (Latest)",
            os.path.join(
                WORKSPACE_DIR,
                "logs/ppo_skrl_rollback_clean-baseline/20261008-150421/26-10-08_15-04-21-052887_TelemetryPPO/checkpoints/agent_3637500.pt",
            ),
        ),
    ]

    # Test sectors:
    # Sector 1: Start Line -> 1000 steps
    # Sector 2: Corner 1 Runway -> 400 steps
    # Sector 4: Intersection -> 400 steps
    sectors_to_test = [
        (1, 1200, "Sector 1: Start Line (Full Runway into Corner 1)"),
        (2, 400, "Sector 2: Corner 1 Runway Approach"),
        (4, 400, "Sector 4: Leg 2 Intersection Approach"),
    ]

    results = []

    for sec_id, max_s, sec_desc in sectors_to_test:
        print(f"\n{'#' * 80}")
        print(f"  TESTING {sec_desc}")
        print(f"{'#' * 80}")
        for ckpt_name, ckpt_path in checkpoints:
            if not os.path.exists(ckpt_path):
                print(f"Skipping {ckpt_name}: not found")
                continue
            load_policy_weights(agent, ckpt_path)
            res = run_single_eval(env, agent, ckpt_name, sector=sec_id, max_steps=max_s)
            results.append(res)

    # Final Summary Table
    print("\n" + "=" * 90)
    print("  DIAGNOSTIC AUDIT RESULTS SUMMARY TABLE")
    print("=" * 90)
    header = f"{'Sector':<10} | {'Checkpoint':<30} | {'Steps':<6} | {'Outcome':<32} | {'Pos':<16} | {'MinW':<7} | {'MinY':<7}"
    print(header)
    print("-" * 90)
    for r in results:
        pos_str = f"({r['final_pos'][0]:.1f}, {r['final_pos'][1]:.1f})"
        outcome_short = r["outcome"][:32]
        print(
            f"Sector {r['sector']:<3} | {r['checkpoint']:<30} | {r['steps']:<6d} | {outcome_short:<32} | {pos_str:<16} | {r['min_dist_w_cm']:<5.1f}cm | {r['min_dist_y_cm']:<5.1f}cm"
        )
    print("=" * 90)

    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
    os._exit(0)
