#!/usr/bin/env python3
"""Automated Checkpoint Regression Auditor for ARCPro RL."""

import argparse
import datetime
import json
import os
import sys

import numpy as np

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
WORKSPACE_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, "../.."))
ARCPRO_LAB_DIR = os.path.join(WORKSPACE_ROOT, "arcproLab")

for _p in (SCRIPT_DIR, WORKSPACE_ROOT, ARCPRO_LAB_DIR):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from isaaclab.app import AppLauncher  # noqa: E402
from sentinel_utils import (  # noqa: E402, F401
    CTA_DB_PATH,
    get_current_run_checkpoint,
    log_to_cta_db,
)

DEFAULT_AUDIT_OUT = f"{WORKSPACE_ROOT}/logs/checkpoint_audits.json"
parser = argparse.ArgumentParser(description="ARCPro RL Checkpoint Regression Auditor")
parser.add_argument("--checkpoint", type=str, default="", help="Checkpoint (.pt) path")
parser.add_argument("--sectors", nargs="+", type=int, default=[1, 2, 3, 4, 5])
parser.add_argument("--steps", type=int, default=200, help="Max steps per sector")
parser.add_argument("--output", type=str, default=DEFAULT_AUDIT_OUT)
parser.add_argument("--auto_heal", action="store_true", help="Auto-adjust curriculum")
parser.add_argument("--min_yellow_thresh", type=float, default=2.5)
parser.add_argument("--min_white_thresh", type=float, default=3.5)
parser.add_argument("--min_avg_speed", type=float, default=0.30)

AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

args_cli.headless = True
args_cli.enable_cameras = True
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import torch  # noqa: E402
from agents.skrl_wrappers import SKRLFlattenWrapper, WarmupActionWrapper  # noqa: E402
from isaaclab.envs import ManagerBasedRLEnv  # noqa: E402
from skrl.agents.torch.ppo import PPO, PPO_DEFAULT_CONFIG  # noqa: E402
from skrl.envs.wrappers.torch import IsaacLabWrapper  # noqa: E402

import arcproLab.mdp.curriculum_config as curr_cfg  # noqa: E402
import arcproLab.mdp.events as mdp_events  # noqa: E402
from arcproLab.agents.skrl_models import ARCProActor, ARCProCritic  # noqa: E402
from arcproLab.arcpro_env_cfg import ARCProEnvCfg  # noqa: E402

find_latest_checkpoint = get_current_run_checkpoint


def set_spawn_sector(sector: int):
    """Force deterministic spawn sector (1 to 5)."""
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
    """Safely loads policy weights into agent."""
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


def _extract_step_metrics(info: dict) -> tuple:
    """Extract speed, white/yellow clearance (cm), waypoints, and lat_err (cm)."""
    spd = float(info["speed_mps"].float().mean().item()) if "speed_mps" in info else 0.0
    dw = float(info["dist_white"][0].item()) * 100.0 if "dist_white" in info else 0.0
    dy = float(info["dist_yellow"][0].item()) * 100.0 if "dist_yellow" in info else 0.0
    wps = (
        float(info["track_wp_delta"].float().mean().item())
        if "track_wp_delta" in info
        else 0.0
    )
    lat = float(info["lat_err"][0].item()) * 100.0 if "lat_err" in info else 0.0
    return spd, dw, dy, wps, lat


def evaluate_single_sector(
    env,
    agent,
    sector: int,
    max_steps: int,
    min_w_thresh: float,
    min_y_thresh: float,
    min_spd_thresh: float,
) -> dict:
    """Evaluates the agent policy in a specific sector for up to max_steps."""
    set_spawn_sector(sector)
    obs, info = env.reset()

    step_count = 0
    speeds, steers, recent_history = [], [], []
    min_dist_w, min_dist_y = float("inf"), float("inf")
    max_wps, done = 0, False
    reason = "SURVIVED"

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
        pos = base_env.scene["robot"].data.root_pos_w[0].cpu().numpy()
        steer_val = float(action[0, 0].item())
        spd, dw, dy, wps, lat = _extract_step_metrics(info)

        speeds.append(spd)
        steers.append(steer_val)
        min_dist_w = min(min_dist_w, dw)
        min_dist_y = min(min_dist_y, dy)
        max_wps = max(max_wps, wps)

        snap = (step_count, float(pos[0]), float(pos[1]), spd, steer_val, dw, dy, lat)
        recent_history.append(snap)
        if len(recent_history) > 5:
            recent_history.pop(0)

        is_term = bool(terminated[0].item())
        is_trunc = bool(truncated[0].item())
        if is_term or is_trunc:
            done = True
            if dw < 4.0:
                reason = f"OUTER_WHITE_HIT (dist_w={dw:.1f}cm < 4.0cm)"
            elif dy < 2.0:
                reason = f"INNER_YELLOW_HIT (dist_y={dy:.1f}cm < 2.0cm)"
            elif abs(lat) > 50.0:
                reason = f"OFF_TRACK (|lat_err|={abs(lat):.1f}cm > 50cm)"
            elif is_trunc:
                reason = "TRUNCATED (Timeout)"
            else:
                reason = "GROUND_CONTACT / EARLY_RESET"

    final_pos = base_env.scene["robot"].data.root_pos_w[0].cpu().numpy()
    avg_spd = float(np.mean(speeds)) if speeds else 0.0
    max_spd = float(np.max(speeds)) if speeds else 0.0
    mean_steer = float(np.mean(steers)) if steers else 0.0

    survived = reason == "SURVIVED"
    w_ok = min_dist_w >= min_w_thresh
    y_ok = min_dist_y >= min_y_thresh
    s_ok = avg_spd >= min_spd_thresh
    passed = survived and w_ok and y_ok and s_ok

    fails = []
    if not survived:
        fails.append(f"Collision reset: {reason}")
    if not w_ok:
        fails.append(f"White clearance {min_dist_w:.1f}cm < {min_w_thresh:.1f}cm")
    if not y_ok:
        fails.append(f"Yellow clearance {min_dist_y:.1f}cm < {min_y_thresh:.1f}cm")
    if not s_ok:
        fails.append(f"Avg speed {avg_spd:.2f}m/s < {min_spd_thresh:.2f}m/s (stalling)")

    keys = ("step", "x", "y", "spd", "steer", "w_cm", "y_cm", "lat_cm")
    hist = [
        {
            k: (s[0] if i == 0 else round(s[i], 2 if i < 5 else 1))
            for i, k in enumerate(keys)
        }
        for s in recent_history
    ]
    return {
        "sector": sector,
        "passed": passed,
        "survived": survived,
        "steps": step_count,
        "max_steps": max_steps,
        "outcome": reason,
        "failure_reasons": fails,
        "final_pos": [round(float(final_pos[0]), 3), round(float(final_pos[1]), 3)],
        "avg_speed": round(avg_spd, 3),
        "max_speed": round(max_spd, 3),
        "mean_steer": round(mean_steer, 3),
        "min_dist_w_cm": round(min_dist_w, 2),
        "min_dist_y_cm": round(min_dist_y, 2),
        "max_wps": int(max_wps),
        "recent_history": hist,
    }


def execute_auto_heal(failed_sectors: list, audit_results: dict) -> list:
    """Dynamically rebalances curriculum weights to auto-heal regressed sectors."""
    cfg = curr_cfg.get_curriculum_config()
    weights = list(cfg.get("sector_weights", [0.20, 0.20, 0.20, 0.20, 0.20]))
    if len(weights) != 5:
        weights = [0.20, 0.20, 0.20, 0.20, 0.20]

    actions = []
    boost = 0.12
    for sec_id in failed_sectors:
        weights[sec_id - 1] += boost
        actions.append(
            f"Boosted Sector {sec_id} curriculum weight (+{boost * 100:.0f}%)"
        )

    total = sum(weights)
    weights = [round(w / total, 3) for w in weights]
    weights[0] = round(weights[0] + (1.0 - sum(weights)), 3)

    updates = {"sector_weights": weights}
    s2 = audit_results.get(2)
    if s2 and not s2["passed"] and s2["max_speed"] > 0.75:
        updates["target_corner_speed"] = 0.35
        actions.append("Enforced target_corner_speed = 0.35 m/s for Corner 1")

    curr_cfg.update_curriculum_config(updates)
    actions.append(f"Updated curriculum weights: {weights}")
    return actions


def log_to_cta(description: str, status: str, files_touched: str, output_summary: str):
    """Log audit action to CTA SQLite DB via shared sentinel_utils helper."""
    log_to_cta_db(
        "AUDIT",
        description,
        status,
        files_touched,
        output_summary,
        "M004",
        "02",
        "Checkpoint Regression Gate",
    )


def main():
    ckpt_path = args_cli.checkpoint or find_latest_checkpoint()
    if not ckpt_path or not os.path.exists(ckpt_path):
        print(f"[Auditor] Error: Checkpoint path does not exist: {ckpt_path}")
        sys.exit(2)

    ckpt_name = os.path.basename(ckpt_path)
    print(
        f"{'=' * 80}\n  ARCPro RL - Automated Checkpoint Regression Auditor\n"
        f"  Target Checkpoint: {ckpt_name}\n  Full Path: {ckpt_path}\n"
        f"  Sectors: {args_cli.sectors} | Steps: {args_cli.steps} | "
        f"Auto-Heal: {args_cli.auto_heal}\n{'=' * 80}"
    )

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
    load_policy_weights(agent, ckpt_path)

    results_by_sector, failed_sectors = {}, []
    for sec_id in args_cli.sectors:
        print(f"\n>>> Auditing Sector {sec_id}...")
        res = evaluate_single_sector(
            env,
            agent,
            sec_id,
            args_cli.steps,
            args_cli.min_white_thresh,
            args_cli.min_yellow_thresh,
            args_cli.min_avg_speed,
        )
        results_by_sector[sec_id] = res
        icon = "✅ PASS" if res["passed"] else "❌ FAIL"
        print(
            f"    Result: {icon} | Steps: {res['steps']}/{res['max_steps']} | "
            f"Spd: {res['avg_speed']:.2f}m/s | MinW: {res['min_dist_w_cm']:.1f}cm | "
            f"MinY: {res['min_dist_y_cm']:.1f}cm"
        )
        if not res["passed"]:
            failed_sectors.append(sec_id)
            print(f"    Failure reasons: {'; '.join(res['failure_reasons'])}")

    env.close()

    tot = len(args_cli.sectors)
    n_pass = tot - len(failed_sectors)
    rate = (n_pass / tot) if tot > 0 else 0.0
    status = "PASSED" if not failed_sectors else "REGRESSION_DETECTED"

    heals = []
    if args_cli.auto_heal and failed_sectors:
        print(f"\n[Auditor] Auto-Healing failed sectors: {failed_sectors}...")
        heals = execute_auto_heal(failed_sectors, results_by_sector)
        for act in heals:
            print(f"  -> {act}")

    now_iso = datetime.datetime.now(datetime.timezone.utc).isoformat()
    record = {
        "timestamp": now_iso,
        "checkpoint": ckpt_path,
        "checkpoint_name": ckpt_name,
        "overall_status": status,
        "pass_rate": round(rate, 3),
        "total_audited": tot,
        "total_passed": n_pass,
        "failed_sectors": failed_sectors,
        "auto_heal_applied": bool(heals),
        "auto_heal_actions": heals,
        "sector_metrics": {str(k): v for k, v in results_by_sector.items()},
    }

    os.makedirs(os.path.dirname(args_cli.output), exist_ok=True)
    history = []
    if os.path.exists(args_cli.output):
        try:
            with open(args_cli.output, "r") as f:
                loaded = json.load(f)
                history = loaded if isinstance(loaded, list) else [loaded]
        except Exception:
            history = []
    history.append(record)
    with open(args_cli.output, "w") as f:
        json.dump(history, f, indent=2)

    print(
        f"\n{'=' * 95}\n  CHECKPOINT REGRESSION AUDIT REPORT: {ckpt_name}\n"
        f"  Overall Status: {status} (Pass Rate: {rate * 100:.1f}% [{n_pass}/{tot}])\n"
        f"{'=' * 95}\n"
        f"{'Sector':<8} | {'Status':<8} | {'Steps':<8} | {'AvgSpd':<9} | "
        f"{'MinW':<8} | {'MinY':<8} | {'Outcome'}\n{'-' * 95}"
    )
    for sec_id in args_cli.sectors:
        r = results_by_sector[sec_id]
        st = "PASS" if r["passed"] else "FAIL"
        s_str = f"{r['steps']}/{r['max_steps']}"
        print(
            f"Sector {sec_id:<2} | {st:<8} | {s_str:<8} | {r['avg_speed']:<6.2f}m/s | "
            f"{r['min_dist_w_cm']:<6.1f}cm | {r['min_dist_y_cm']:<6.1f}cm | {r['outcome'][:35]}"
        )
    print("=" * 95)

    desc = f"Audit of {ckpt_name}: {status} ({n_pass}/{tot} sectors passed)"
    out_sum = f"Pass rate: {rate * 100:.1f}%. Auto-heal: {'; '.join(heals)}"
    log_to_cta(desc, "SUCCESS" if status == "PASSED" else "WARNING", ckpt_path, out_sum)
    sys.exit(0 if status == "PASSED" else 1)


if __name__ == "__main__":
    main()
    simulation_app.close()
    os._exit(0)
