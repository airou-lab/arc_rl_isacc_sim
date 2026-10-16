#!/usr/bin/env python3
"""
Heavy Invasive Multi-Sector Checkpoint Audit Suite (Deterministic Sector Isolation):
Compares landmark champion agent_3587500_c1_champion.pt against overnight checkpoints
(agent_3687500.pt and agent_3737500.pt) across all 5 track sectors:
  Sector 1: Start Line & West Runway -> Corner 1 (600 steps)
  Sector 2: Corner 1 Runway Approach -> Corner 1 (400 steps)
  Sector 3: Corner 1 Apex Turn (Re-centered Arc) -> Leg 2 (400 steps)
  Sector 4: Leg 2 Intersection Approach -> North Straight (400 steps)
  Sector 5: North Straightaway -> Corner 4 (400 steps)

Dynamically writes curriculum_sectors.json to guarantee 100% deterministic
spawns in the target sector. Invasively probes IsaacLab TerminationManager
to record exact termination terms, boundary clearances, and trajectory histories.
"""

import sys
import os
import json
import time

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
from arcproLab.agents.skrl_wrappers import SKRLFlattenWrapper
from skrl.envs.wrappers.torch import IsaacLabWrapper
from skrl.agents.torch.ppo import PPO, PPO_DEFAULT_CONFIG
import arcproLab.mdp.curriculum_config as curr_cfg


class WarmupActionWrapper(gym.Wrapper):
    def __init__(self, env, warmup_steps=10):
        super().__init__(env)
        self.warmup_steps = warmup_steps

    def step(self, action):
        if hasattr(self.env.unwrapped, "episode_length_buf"):
            mask = self.env.unwrapped.episode_length_buf < self.warmup_steps
            action = action.clone()
            action[mask, 0] = 0.0  # Neutral steering
            action[mask, 1] = -1.0  # Zero throttle
        return self.env.step(action)


CURRICULUM_PATH = os.path.join(ARCPRO_LAB_DIR, "mdp", "curriculum_sectors.json")


def set_deterministic_spawn_sector(sector: int):
    """Write sector_weights directly into curriculum_sectors.json to force target sector."""
    weights = [0.0, 0.0, 0.0, 0.0, 0.0]
    weights[sector - 1] = 1.0
    with open(CURRICULUM_PATH, "r") as f:
        cfg = json.load(f)
    cfg["sector_weights"] = weights
    cfg["updated_at"] = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
    with open(CURRICULUM_PATH, "w") as f:
        json.dump(cfg, f, indent=2)
    # Reload in-memory config immediately
    curr_cfg.get_curriculum_config()


def restore_curriculum_sectors():
    """Restore uniform 20% weights across all 5 sectors."""
    weights = [0.20, 0.20, 0.20, 0.20, 0.20]
    with open(CURRICULUM_PATH, "r") as f:
        cfg = json.load(f)
    cfg["sector_weights"] = weights
    cfg["updated_at"] = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
    with open(CURRICULUM_PATH, "w") as f:
        json.dump(cfg, f, indent=2)
    curr_cfg.get_curriculum_config()


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


def run_invasive_episode(env, agent, sector: int, max_steps: int):
    set_deterministic_spawn_sector(sector)
    obs, info = env.reset()

    base_env = env.unwrapped
    step_count = 0
    speeds = []
    steers = []
    lat_errs = []
    min_dist_w = float("inf")
    min_dist_y = float("inf")
    max_wps = 0
    done = False
    termination_reason = "SURVIVED (Max Steps)"

    # Track pre-reset states
    pre_pos = base_env.scene["robot"].data.root_pos_w[0].cpu().numpy().copy()
    pre_dist_w = 0.0
    pre_dist_y = 0.0
    pre_lat_err = 0.0
    pre_speed = 0.0
    pre_steer = 0.0

    history = []

    while step_count < max_steps and not done:
        with torch.no_grad():
            _, _, outputs = agent.act(obs, timestep=0, timesteps=0)
            action = outputs.get("mean_actions", None)
            if action is None:
                action = agent.policy.compute({"states": obs})[0]
            action = action.clone()

        # Capture robot state BEFORE env.step() computes terminations and resets
        pre_pos = base_env.scene["robot"].data.root_pos_w[0].cpu().numpy().copy()
        pre_steer = float(action[0, 0].item())
        pre_speed = (
            float(info["speed_mps"].float().mean().item())
            if "speed_mps" in info
            else 0.0
        )
        pre_dist_w = (
            float(info["dist_white"][0].item()) * 100.0 if "dist_white" in info else 0.0
        )
        pre_dist_y = (
            float(info["dist_yellow"][0].item()) * 100.0
            if "dist_yellow" in info
            else 0.0
        )
        pre_lat_err = (
            float(info["lat_err"][0].item()) * 100.0 if "lat_err" in info else 0.0
        )
        wps_val = (
            float(info["track_wp_delta"].float().mean().item())
            if "track_wp_delta" in info
            else 0.0
        )

        speeds.append(pre_speed)
        steers.append(pre_steer)
        lat_errs.append(abs(pre_lat_err))
        min_dist_w = min(min_dist_w, pre_dist_w)
        min_dist_y = min(min_dist_y, pre_dist_y)
        max_wps = max(max_wps, wps_val)

        step_snapshot = {
            "step": step_count,
            "x": float(pre_pos[0]),
            "y": float(pre_pos[1]),
            "speed": pre_speed,
            "steer": pre_steer,
            "dist_w": pre_dist_w,
            "dist_y": pre_dist_y,
            "lat_err": pre_lat_err,
        }
        history.append(step_snapshot)
        if len(history) > 10:
            history.pop(0)

        obs, reward, terminated, truncated, info = env.step(action)
        step_count += 1

        is_term = bool(terminated[0].item())
        is_trunc = bool(truncated[0].item())

        if is_term or is_trunc:
            done = True
            # Invasively diagnose the exact termination mechanism from pre-reset telemetry
            if is_trunc:
                termination_reason = "TRUNCATED (Timeout)"
            elif pre_dist_w < 5.0:
                termination_reason = f"OUTER WHITE BREACH (W={pre_dist_w:.1f}cm < 5cm)"
            elif pre_dist_y < 3.0:
                termination_reason = f"INNER YELLOW BREACH (Y={pre_dist_y:.1f}cm < 3cm)"
            elif abs(pre_lat_err) > 40.0:
                termination_reason = (
                    f"OFF-TRACK ENVELOPE (|LatErr|={abs(pre_lat_err):.1f}cm > 40cm)"
                )
            elif pre_pos[0] > -2.08 and -7.5 <= pre_pos[1] <= -2.4:
                termination_reason = (
                    f"EAST CLIFF WASHOUT (X={pre_pos[0]:.2f}m > -2.08m)"
                )
            else:
                # Check IsaacLab termination manager terms
                tm = getattr(base_env, "termination_manager", None)
                term_desc = []
                if tm is not None and hasattr(tm, "_terms"):
                    for t_name, term_obj in tm._terms.items():
                        try:
                            t_done = (
                                term_obj.func(base_env, **term_obj.params)
                                if hasattr(term_obj, "params")
                                else term_obj.func(base_env)
                            )
                            if bool(t_done[0].item()):
                                term_desc.append(t_name)
                        except Exception:
                            pass
                if term_desc:
                    termination_reason = f"TRIGGERED: {', '.join(term_desc)}"
                else:
                    termination_reason = "ROADMARK / GEOMETRIC BOUNDARY"

    return {
        "steps": step_count,
        "max_steps": max_steps,
        "survived": (step_count >= max_steps),
        "outcome": termination_reason,
        "pre_crash_pos": (float(pre_pos[0]), float(pre_pos[1])),
        "avg_speed": float(np.mean(speeds)) if speeds else 0.0,
        "max_speed": float(np.max(speeds)) if speeds else 0.0,
        "avg_steer": float(np.mean(steers)) if steers else 0.0,
        "steer_std": float(np.std(steers)) if steers else 0.0,
        "min_dist_w_cm": min_dist_w,
        "min_dist_y_cm": min_dist_y,
        "mean_lat_err_cm": float(np.mean(lat_errs)) if lat_errs else 0.0,
        "max_lat_err_cm": float(np.max(lat_errs)) if lat_errs else 0.0,
        "max_wps": max_wps,
        "history": history,
    }


def main():
    print("=" * 95)
    print(
        "  HEAVY INVASIVE MULTI-SECTOR CHECKPOINT AUDIT (DETERMINISTIC SECTOR ISOLATION)"
    )
    print("=" * 95)

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
            "Champion (3.58M)",
            os.path.join(
                WORKSPACE_DIR,
                "archive/landmark_checkpoints/agent_3587500_c1_champion.pt",
            ),
        ),
        (
            "Previous (4.03M)",
            os.path.join(
                WORKSPACE_DIR,
                "logs/ppo_skrl_rollback_clean-baseline/20261011-201850/26-10-11_20-18-50-904988_TelemetryPPO/checkpoints/agent_4037500.pt",
            ),
        ),
        (
            "Fresh (4.13M)",
            os.path.join(
                WORKSPACE_DIR,
                "logs/ppo_skrl_rollback_clean-baseline/20261012-015610/26-10-12_01-56-10-146750_TelemetryPPO/checkpoints/agent_4137500.pt",
            ),
        ),
    ]

    # 5 Sector Audit Matrix (Exact Deterministic Spawns)
    sectors = [
        (1, 600, "Sector 1: Start Line (West Runway Southbound)"),
        (2, 400, "Sector 2: Corner 1 Runway Approach"),
        (3, 400, "Sector 3: Leg 2 Post-C1 Exit & Runway"),
        (4, 400, "Sector 4: North Straight & Sweeping Bend"),
        (5, 400, "Sector 5: Top North Straight & Corner 4 Approach"),
    ]

    audit_results = []

    try:
        for sec_id, max_s, sec_desc in sectors:
            print(f"\n{'#' * 95}")
            print(f"  AUDITING {sec_desc} (Max Steps: {max_s})")
            print(f"{'#' * 95}")

            for ckpt_label, ckpt_path in checkpoints:
                if not os.path.exists(ckpt_path):
                    print(f"  [WARN] Checkpoint not found: {ckpt_path}")
                    continue

                load_policy_weights(agent, ckpt_path)

                ep_results = []
                for ep in range(2):
                    res = run_invasive_episode(
                        env, agent, sector=sec_id, max_steps=max_s
                    )
                    ep_results.append(res)

                avg_steps = np.mean([r["steps"] for r in ep_results])
                surv_rate = (
                    np.mean([1.0 if r["survived"] else 0.0 for r in ep_results]) * 100.0
                )
                min_w = min([r["min_dist_w_cm"] for r in ep_results])
                min_y = min([r["min_dist_y_cm"] for r in ep_results])
                avg_spd = np.mean([r["avg_speed"] for r in ep_results])
                max_wps = max([r["max_wps"] for r in ep_results])
                outcomes = [r["outcome"] for r in ep_results]

                record = {
                    "sector": sec_id,
                    "sector_desc": sec_desc,
                    "ckpt_label": ckpt_label,
                    "ep_results": ep_results,
                    "avg_steps": avg_steps,
                    "surv_rate": surv_rate,
                    "min_w": min_w,
                    "min_y": min_y,
                    "avg_spd": avg_spd,
                    "max_wps": max_wps,
                    "outcomes": outcomes,
                }
                audit_results.append(record)

                status_tag = (
                    "[100% PASS]" if surv_rate == 100.0 else f"[{surv_rate:.0f}% SURV]"
                )
                print(
                    f"  {status_tag} {ckpt_label:<24}: Avg Steps={avg_steps:.0f}/{max_s} | Spd={avg_spd:.2f}m/s | MinW={min_w:.1f}cm | MinY={min_y:.1f}cm | WPs={max_wps:.0f}"
                )
                for ep_i, r in enumerate(ep_results):
                    pos = r["pre_crash_pos"]
                    print(
                        f"    Ep {ep_i + 1}: {r['outcome']} @ Step {r['steps']} | Pos=({pos[0]:.2f}, {pos[1]:.2f}) | LatErr={r['mean_lat_err_cm']:.1f}cm"
                    )
                    if not r["survived"] and r["history"]:
                        print("          Pre-crash 3-step trail:")
                        for h in r["history"][-3:]:
                            print(
                                f"            Step {h['step']:3d}: Pos=({h['x']:.2f}, {h['y']:.2f}) | Spd={h['speed']:.2f}m/s | Steer={h['steer']:+.2f} | W={h['dist_w']:4.1f}cm | Y={h['dist_y']:4.1f}cm | LatErr={h['lat_err']:+.1f}cm"
                            )

        # Master Summary Table
        print("\n" + "=" * 110)
        print("  HEAVY INVASIVE AUDIT MASTER COMPARISON TABLE")
        print("=" * 110)
        header = f"{'Sector':<10} | {'Checkpoint':<24} | {'Surv%':<6} | {'Avg Steps':<10} | {'Speed':<8} | {'Min W':<7} | {'Min Y':<7} | {'Primary Outcome':<28}"
        print(header)
        print("-" * 110)
        for r in audit_results:
            primary_out = r["outcomes"][0][:28]
            row = f"Sector {r['sector']:<3} | {r['ckpt_label']:<24} | {r['surv_rate']:>4.0f}% | {r['avg_steps']:>7.1f}   | {r['avg_spd']:>5.2f}m/s | {r['min_w']:>5.1f}cm | {r['min_y']:>5.1f}cm | {primary_out:<28}"
            print(row)
        print("=" * 110)
    finally:
        restore_curriculum_sectors()
        env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
    os._exit(0)
