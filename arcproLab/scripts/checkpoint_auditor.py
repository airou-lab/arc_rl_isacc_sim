#!/usr/bin/env python3
"""
Automated Checkpoint Regression Auditor for ARCPro RL.

Evaluates an RL checkpoint across all 5 certified track sectors in headless mode,
determines pass/fail based on boundary clearance, speed, and collision-free transit,
and optionally triggers auto-healing interventions (curriculum weight adjustments)
to prevent sector regressions from persisting undetected.
"""

import argparse
import datetime
import json
import os
import sys
import numpy as np

# Set workspace paths
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
WORKSPACE_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, "../.."))
ARCPRO_LAB_DIR = os.path.join(WORKSPACE_ROOT, "arcproLab")

if WORKSPACE_ROOT not in sys.path:
    sys.path.insert(0, WORKSPACE_ROOT)
if ARCPRO_LAB_DIR not in sys.path:
    sys.path.insert(0, ARCPRO_LAB_DIR)

from isaaclab.app import AppLauncher

# CLI arguments parsed through AppLauncher
parser = argparse.ArgumentParser(description="ARCPro RL Automated Checkpoint Regression Auditor")
parser.add_argument("--checkpoint", type=str, default="", help="Path to checkpoint (.pt) to audit. If omitted, uses latest checkpoint.")
parser.add_argument("--sectors", nargs="+", type=int, default=[1, 2, 3, 4, 5], help="Track sectors to evaluate (1 to 5).")
parser.add_argument("--steps", type=int, default=200, help="Maximum evaluation steps per sector (default: 200).")
parser.add_argument("--output", type=str, default=os.path.join(WORKSPACE_ROOT, "logs/checkpoint_audits.json"), help="Output JSON log file path.")
parser.add_argument("--auto_heal", action="store_true", help="Automatically adjust curriculum weights if regressions are detected.")
parser.add_argument("--min_yellow_thresh", type=float, default=2.5, help="Minimum yellow clearance in cm before failing (default: 2.5).")
parser.add_argument("--min_white_thresh", type=float, default=3.5, help="Minimum white clearance in cm before failing (default: 3.5).")
parser.add_argument("--min_avg_speed", type=float, default=0.30, help="Minimum average speed in m/s before failing (default: 0.30).")

AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

# Launch headless Isaac Sim app
args_cli.headless = True
args_cli.enable_cameras = True
app_launcher = AppLauncher(args_cli)
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

CTA_DB_PATH = os.path.join(WORKSPACE_ROOT, ".cta/cta_turns.db")


class WarmupActionWrapper(gym.Wrapper):
    """Zeroes out actions during the initial simulation warmup steps."""
    def __init__(self, env, warmup_steps=10):
        super().__init__(env)
        self.warmup_steps = warmup_steps

    def step(self, action):
        if hasattr(self.env.unwrapped, "episode_length_buf"):
            mask = self.env.unwrapped.episode_length_buf < self.warmup_steps
            action = action.clone()
            action[mask, 0] = 0.0  # Steer: centered
            action[mask, 1] = -1.0 # Throttle: brake/stop
        return self.env.step(action)


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


def find_latest_checkpoint() -> str:
    """Finds the newest saved checkpoint in logs directory."""
    candidate_dirs = [
        os.path.join(WORKSPACE_ROOT, "logs/ppo_skrl_rollback_clean-baseline"),
        os.path.join(WORKSPACE_ROOT, "logs/ppo_skrl"),
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


def evaluate_single_sector(env, agent, sector: int, max_steps: int, min_w_thresh: float, min_y_thresh: float, min_spd_thresh: float) -> dict:
    """Evaluates the agent policy in a specific sector for up to max_steps."""
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

        speeds.append(speed_val)
        steers.append(steer_val)
        min_dist_w = min(min_dist_w, dist_w_cm)
        min_dist_y = min(min_dist_y, dist_y_cm)
        max_wps = max(max_wps, wps_val)

        step_snapshot = (
            step_count,
            float(robot_pos[0]),
            float(robot_pos[1]),
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
            if dist_w_cm < 4.0:
                termination_reason = f"OUTER_WHITE_HIT (dist_w={dist_w_cm:.1f}cm < 4.0cm)"
            elif dist_y_cm < 2.0:
                termination_reason = f"INNER_YELLOW_HIT (dist_y={dist_y_cm:.1f}cm < 2.0cm)"
            elif abs(lat_err_cm) > 50.0:
                termination_reason = f"OFF_TRACK (|lat_err|={abs(lat_err_cm):.1f}cm > 50cm)"
            elif is_trunc:
                termination_reason = "TRUNCATED (Timeout)"
            else:
                termination_reason = "GROUND_CONTACT / EARLY_RESET"

    final_pos = base_env.scene["robot"].data.root_pos_w[0].cpu().numpy()
    avg_speed = float(np.mean(speeds)) if speeds else 0.0
    max_speed = float(np.max(speeds)) if speeds else 0.0
    mean_steer = float(np.mean(steers)) if steers else 0.0

    # Sector passing criteria:
    # 1. Did not trigger early collision termination
    # 2. Maintained safe boundary clearance
    # 3. Maintained reasonable operational speed
    survived = (termination_reason == "SURVIVED")
    white_ok = (min_dist_w >= min_w_thresh)
    yellow_ok = (min_dist_y >= min_y_thresh)
    speed_ok = (avg_speed >= min_spd_thresh)
    passed = survived and white_ok and yellow_ok and speed_ok

    failure_reasons = []
    if not survived:
        failure_reasons.append(f"Collision reset: {termination_reason}")
    if not white_ok:
        failure_reasons.append(f"White clearance {min_dist_w:.1f}cm < {min_w_thresh:.1f}cm")
    if not yellow_ok:
        failure_reasons.append(f"Yellow clearance {min_dist_y:.1f}cm < {min_y_thresh:.1f}cm")
    if not speed_ok:
        failure_reasons.append(f"Avg speed {avg_speed:.2f}m/s < {min_spd_thresh:.2f}m/s (stalling)")

    return {
        "sector": sector,
        "passed": passed,
        "survived": survived,
        "steps": step_count,
        "max_steps": max_steps,
        "outcome": termination_reason,
        "failure_reasons": failure_reasons,
        "final_pos": [round(float(final_pos[0]), 3), round(float(final_pos[1]), 3)],
        "avg_speed": round(avg_speed, 3),
        "max_speed": round(max_speed, 3),
        "mean_steer": round(mean_steer, 3),
        "min_dist_w_cm": round(min_dist_w, 2),
        "min_dist_y_cm": round(min_dist_y, 2),
        "max_wps": int(max_wps),
        "recent_history": [
            {
                "step": s[0],
                "x": round(s[1], 2),
                "y": round(s[2], 2),
                "spd": round(s[3], 2),
                "steer": round(s[4], 2),
                "w_cm": round(s[5], 1),
                "y_cm": round(s[6], 1),
                "lat_cm": round(s[7], 1),
            }
            for s in recent_history
        ],
    }


def execute_auto_heal(failed_sectors: list, audit_results: dict) -> list:
    """Dynamically rebalances curriculum weights and parameters to auto-heal regressed sectors."""
    cfg = curr_cfg.get_curriculum_config()
    current_weights = list(cfg.get("sector_weights", [0.20, 0.20, 0.20, 0.20, 0.20]))
    if len(current_weights) != 5:
        current_weights = [0.20, 0.20, 0.20, 0.20, 0.20]

    actions_taken = []
    new_weights = list(current_weights)

    # For each failed sector, increase its curriculum priority
    boost_per_failure = 0.12
    total_boost = 0.0

    for sec_id in failed_sectors:
        idx = sec_id - 1
        new_weights[idx] += boost_per_failure
        total_boost += boost_per_failure
        actions_taken.append(f"Boosted Sector {sec_id} curriculum weight (+{boost_per_failure*100:.0f}%)")

    # Normalize weights so they sum exactly to 1.0
    total = sum(new_weights)
    new_weights = [round(w / total, 3) for w in new_weights]
    # Adjust last element for exact 1.0 sum
    diff = 1.0 - sum(new_weights)
    new_weights[0] = round(new_weights[0] + diff, 3)

    updates = {
        "sector_weights": new_weights,
    }

    # If Sector 1 or Sector 2 failed with high speed washout, ensure corner speed target is conservative
    s2_result = audit_results.get(2)
    if s2_result and not s2_result["passed"]:
        if s2_result["max_speed"] > 0.75:
            updates["target_corner_speed"] = 0.35
            actions_taken.append("Enforced target_corner_speed = 0.35 m/s to combat understeer washout")

    curr_cfg.update_curriculum_config(updates)
    actions_taken.append(f"Updated curriculum weights: {new_weights}")
    return actions_taken


def log_to_cta(description: str, status: str, files_touched: str, output_summary: str):
    """Log audit action to CTA SQLite DB if available."""
    if not os.path.exists(CTA_DB_PATH):
        return
    try:
        import sqlite3
        conn = sqlite3.connect(CTA_DB_PATH, timeout=5.0)
        c = conn.cursor()
        now_utc = datetime.datetime.now(datetime.timezone.utc).isoformat()
        session_id = f"auditor_{int(datetime.datetime.now().timestamp())}"
        c.execute(
            """INSERT INTO turn_actions (session_id, timestamp, milestone, phase, task,
                                         action_type, description, status, files_touched, output_summary)
               VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?)""",
            (
                session_id,
                now_utc,
                "M004",
                "02",
                "Checkpoint Regression Gate",
                "AUDIT",
                description,
                status,
                files_touched,
                output_summary,
            ),
        )
        conn.commit()
        conn.close()
    except Exception as e:
        print(f"[Auditor] Warning: could not write to CTA DB: {e}")


def main():
    ckpt_path = args_cli.checkpoint
    if not ckpt_path:
        ckpt_path = find_latest_checkpoint()

    if not ckpt_path or not os.path.exists(ckpt_path):
        print(f"[Auditor] Error: Checkpoint path does not exist: {ckpt_path}")
        sys.exit(2)

    ckpt_name = os.path.basename(ckpt_path)
    print("=" * 80)
    print("  ARCPro RL - Automated Checkpoint Regression Auditor")
    print(f"  Target Checkpoint: {ckpt_name}")
    print(f"  Full Path: {ckpt_path}")
    print(f"  Sectors to Audit: {args_cli.sectors}")
    print(f"  Steps per Sector: {args_cli.steps}")
    print(f"  Auto-Heal Enabled: {args_cli.auto_heal}")
    print("=" * 80)

    # 1. Build Environment
    env_cfg = ARCProEnvCfg()
    env_cfg.scene.num_envs = 1
    env_cfg.enable_cameras = True
    env_cfg.__post_init__()

    env = ManagerBasedRLEnv(cfg=env_cfg)
    env = WarmupActionWrapper(env, warmup_steps=10)
    env = IsaacLabWrapper(env)
    env = SKRLFlattenWrapper(env)

    # 2. Build Policy Agent
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

    # 3. Load Checkpoint
    load_policy_weights(agent, ckpt_path)

    # 4. Audit each sector
    results_by_sector = {}
    failed_sectors = []

    for sec_id in args_cli.sectors:
        print(f"\n>>> Auditing Sector {sec_id}...")
        res = evaluate_single_sector(
            env=env,
            agent=agent,
            sector=sec_id,
            max_steps=args_cli.steps,
            min_w_thresh=args_cli.min_white_thresh,
            min_y_thresh=args_cli.min_yellow_thresh,
            min_spd_thresh=args_cli.min_avg_speed,
        )
        results_by_sector[sec_id] = res
        status_icon = "✅ PASS" if res["passed"] else "❌ FAIL"
        print(f"    Result: {status_icon} | Steps: {res['steps']}/{res['max_steps']} | Spd: {res['avg_speed']:.2f}m/s | MinW: {res['min_dist_w_cm']:.1f}cm | MinY: {res['min_dist_y_cm']:.1f}cm")
        if not res["passed"]:
            failed_sectors.append(sec_id)
            print(f"    Failure reasons: {'; '.join(res['failure_reasons'])}")

    env.close()

    total_audited = len(args_cli.sectors)
    total_passed = total_audited - len(failed_sectors)
    pass_rate = (total_passed / total_audited) if total_audited > 0 else 0.0
    overall_status = "PASSED" if len(failed_sectors) == 0 else "REGRESSION_DETECTED"

    # Auto-heal execution if requested
    auto_heal_actions = []
    if args_cli.auto_heal and failed_sectors:
        print(f"\n[Auditor] Triggering Auto-Healing for failed sectors: {failed_sectors}...")
        auto_heal_actions = execute_auto_heal(failed_sectors, results_by_sector)
        for act in auto_heal_actions:
            print(f"  -> {act}")

    # Build structured audit record
    audit_record = {
        "timestamp": datetime.datetime.now(datetime.timezone.utc).isoformat(),
        "checkpoint": ckpt_path,
        "checkpoint_name": ckpt_name,
        "overall_status": overall_status,
        "pass_rate": round(pass_rate, 3),
        "total_audited": total_audited,
        "total_passed": total_passed,
        "failed_sectors": failed_sectors,
        "auto_heal_applied": bool(auto_heal_actions),
        "auto_heal_actions": auto_heal_actions,
        "sector_metrics": {str(k): v for k, v in results_by_sector.items()},
    }

    # Save to JSON log file
    os.makedirs(os.path.dirname(args_cli.output), exist_ok=True)
    history = []
    if os.path.exists(args_cli.output):
        try:
            with open(args_cli.output, "r") as f:
                history = json.load(f)
                if not isinstance(history, list):
                    history = [history]
        except Exception:
            history = []
    history.append(audit_record)
    with open(args_cli.output, "w") as f:
        json.dump(history, f, indent=2)

    # Print Summary Table
    print("\n" + "=" * 95)
    print(f"  CHECKPOINT REGRESSION AUDIT REPORT: {ckpt_name}")
    print(f"  Overall Status: {overall_status} (Pass Rate: {pass_rate*100:.1f}% [{total_passed}/{total_audited}])")
    print("=" * 95)
    header = f"{'Sector':<8} | {'Status':<8} | {'Steps':<8} | {'AvgSpd':<9} | {'MinW':<8} | {'MinY':<8} | {'Outcome'}"
    print(header)
    print("-" * 95)
    for sec_id in args_cli.sectors:
        r = results_by_sector[sec_id]
        st = "PASS" if r["passed"] else "FAIL"
        steps_str = f"{r['steps']}/{r['max_steps']}"
        outcome_str = r["outcome"][:35]
        print(f"Sector {sec_id:<2} | {st:<8} | {steps_str:<8} | {r['avg_speed']:<6.2f}m/s | {r['min_dist_w_cm']:<6.1f}cm | {r['min_dist_y_cm']:<6.1f}cm | {outcome_str}")
    print("=" * 95)

    # Log to CTA database
    summary_desc = f"Audit of {ckpt_name}: {overall_status} ({total_passed}/{total_audited} sectors passed)"
    log_to_cta(
        description=summary_desc,
        status="SUCCESS" if overall_status == "PASSED" else "WARNING",
        files_touched=ckpt_path,
        output_summary=f"Pass rate: {pass_rate*100:.1f}%. Auto-heal: {'; '.join(auto_heal_actions)}",
    )

    exit_code = 0 if overall_status == "PASSED" else 1
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
    simulation_app.close()
    os._exit(0)
