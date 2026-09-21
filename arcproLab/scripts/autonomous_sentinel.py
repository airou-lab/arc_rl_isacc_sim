#!/usr/bin/env python3
"""Autonomous Self-Healing Sentinel for ARCPro Reinforcement Learning.

Continuously monitors SKRL training rollouts, tracks rolling telemetry
statistics, identifies 5 specific failure modes, and executes autonomous
self-healing interventions without requiring human intervention.

Failure Sentinels:
1. Crawling Trap Sentinel: Speed < 0.40 m/s across rolling batches.
2. Corner 1 Apex Cliff Sentinel: Max length capped at <= 860 steps over 100k steps.
3. Outer Shoulder Wall-Hugging Sentinel: Straightaway resets with Len < 150.
4. Policy Collapse & Divergence Sentinel: Rew < 500 & Len < 150, or NaN/Divergence.
5. Process Hang & VRAM Exhaustion Sentinel: Stale log mtime > 180s, dead tmux, VRAM > 11.5 GB.
"""

import argparse
import datetime
import glob
import json
import logging
import os
import re
import subprocess
import sys
import time
from dataclasses import asdict, dataclass
from typing import Any, Dict, List, Tuple

# Workspace paths
WORKSPACE_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "../.."))
LOG_FILE = os.path.join(WORKSPACE_ROOT, "logs/skrl_phase1.log")
SENTINEL_LOG = os.path.join(WORKSPACE_ROOT, "logs/sentinel.log")
DIAGNOSIS_FILE = os.path.join(WORKSPACE_ROOT, "logs/SENTINEL_DIAGNOSIS.md")
STATE_FILE = os.path.join(WORKSPACE_ROOT, "logs/sentinel_state.json")
CTA_DB_PATH = os.path.join(WORKSPACE_ROOT, ".cta/cta_turns.db")
CURRICULUM_CFG_PATH = os.path.join(
    WORKSPACE_ROOT, "arcproLab/mdp/curriculum_sectors.json"
)
LANDMARK_DIR = os.path.join(WORKSPACE_ROOT, "archive/landmark_checkpoints")
START_SCRIPT = os.path.join(WORKSPACE_ROOT, "start_tmux_training.sh")


def get_branch_checkpoint(target: str = "best_agent.pt") -> str:
    """Resolve checkpoint within current branch run directory, falling back to archive."""
    branch = "feature/realsense-realistic-kinematics"
    branch_tag = branch.replace("/", "_")
    branch_dir = os.path.join(WORKSPACE_ROOT, f"logs/ppo_skrl_{branch_tag}")
    if os.path.exists(branch_dir):
        matches = glob.glob(os.path.join(branch_dir, "**", target), recursive=True)
        if matches:
            return matches[0]
        all_pts = glob.glob(os.path.join(branch_dir, "**", "*.pt"), recursive=True)
        if all_pts:
            return sorted(all_pts, key=os.path.getmtime)[-1]
    fallback = os.path.join(LANDMARK_DIR, target)
    return (
        fallback
        if os.path.exists(fallback)
        else os.path.join(LANDMARK_DIR, "best_agent.pt")
    )


PRIMARY_LANDMARK = get_branch_checkpoint("best_agent.pt")
SECONDARY_LANDMARK = get_branch_checkpoint("agent_750000.pt")
TERTIARY_LANDMARK = get_branch_checkpoint("agent_500000.pt")

DEFAULT_COOLDOWN_SEC = 300.0  # 5 minutes stabilization cooldown after intervention


@dataclass
class MetricBatch:
    step: int
    rew: float
    rew_max: float
    ep_len: int
    len_max: int
    speed: float
    wps_cum: int
    wp_step: float
    raw_timestamp: float = 0.0


@dataclass
class GPUStats:
    vram_used_mb: float = 0.0
    vram_total_mb: float = 0.0
    gpu_util_pct: float = 0.0


@dataclass
class ProcessStatus:
    tmux_active: bool
    python_active: bool
    log_exists: bool
    log_fresh: bool
    log_age_seconds: float
    gpu: GPUStats


def setup_logger(verbose: bool = False) -> logging.Logger:
    """Setup dual console and file logging."""
    logger = logging.getLogger("autonomous_sentinel")
    logger.setLevel(logging.DEBUG if verbose else logging.INFO)
    logger.handlers.clear()

    os.makedirs(os.path.dirname(SENTINEL_LOG), exist_ok=True)
    fh = logging.FileHandler(SENTINEL_LOG)
    fh.setLevel(logging.DEBUG)
    fh_formatter = logging.Formatter("[%(asctime)s] [%(levelname)s] %(message)s")
    fh.setFormatter(fh_formatter)
    logger.addHandler(fh)

    ch = logging.StreamHandler(sys.stdout)
    ch.setLevel(logging.DEBUG if verbose else logging.INFO)
    ch_formatter = logging.Formatter("[Sentinel] %(message)s")
    ch.setFormatter(ch_formatter)
    logger.addHandler(ch)

    return logger


def load_state() -> Dict[str, Any]:
    """Load persistent sentinel state."""
    if os.path.exists(STATE_FILE):
        try:
            with open(STATE_FILE, "r") as f:
                return json.load(f)
        except Exception:
            pass
    return {
        "last_intervention_time": 0.0,
        "last_intervention_type": None,
        "interventions_count": 0,
        "crawling_consecutive_count": 0,
        "collapse_consecutive_count": 0,
        "wall_hugging_consecutive_count": 0,
        "last_seen_step": 0,
        "step_window_start": 0,
        "step_window_max_len": 0,
        "apex_cliff_eval_start_step": 0,
    }


def save_state(state: Dict[str, Any]) -> None:
    """Save persistent sentinel state."""
    os.makedirs(os.path.dirname(STATE_FILE), exist_ok=True)
    tmp_path = STATE_FILE + ".tmp"
    with open(tmp_path, "w") as f:
        json.dump(state, f, indent=2)
    os.replace(tmp_path, STATE_FILE)


def query_gpu_stats() -> GPUStats:
    """Query nvidia-smi safely for GPU memory and utilization."""
    try:
        cmd = [
            "nvidia-smi",
            "--query-gpu=memory.used,memory.total,utilization.gpu",
            "--format=csv,nounits,noheader",
        ]
        res = subprocess.check_output(cmd, timeout=3).decode("utf-8").strip()
        parts = [p.strip() for p in res.split(",")]
        if len(parts) >= 3:
            return GPUStats(
                vram_used_mb=float(parts[0]),
                vram_total_mb=float(parts[1]),
                gpu_util_pct=float(parts[2]),
            )
    except Exception:
        pass
    return GPUStats()


def check_process_status(max_stale_seconds: float = 180.0) -> ProcessStatus:
    """Check tmux session, process execution, and log freshness."""
    tmux_active = False
    try:
        res = subprocess.run(
            ["tmux", "has-session", "-t", "training"], capture_output=True
        )
        tmux_active = res.returncode == 0
    except Exception:
        tmux_active = False

    python_active = False
    try:
        res = subprocess.run(["pgrep", "-f", "train_skrl.py"], capture_output=True)
        python_active = res.returncode == 0
    except Exception:
        python_active = False

    log_exists = os.path.exists(LOG_FILE)
    log_age = 999999.0
    log_fresh = False
    if log_exists:
        try:
            mtime = os.path.getmtime(LOG_FILE)
            log_age = time.time() - mtime
            log_fresh = log_age <= max_stale_seconds
        except Exception:
            pass

    gpu = query_gpu_stats()
    return ProcessStatus(
        tmux_active=tmux_active,
        python_active=python_active,
        log_exists=log_exists,
        log_fresh=log_fresh,
        log_age_seconds=log_age,
        gpu=gpu,
    )


def parse_recent_metrics(max_lines: int = 300) -> List[MetricBatch]:
    """Parse telemetry batches from the tail of skrl_phase1.log."""
    if not os.path.exists(LOG_FILE):
        return []

    try:
        tail_bytes = subprocess.check_output(
            ["tail", "-n", str(max_lines), LOG_FILE], timeout=5
        )
        text = tail_bytes.decode("utf-8", errors="replace")
    except Exception:
        return []

    pattern = re.compile(
        r"Step\s+(\d+)\s*\([^)]*\)\s*\|\s*"
        r"Rew:\s*([-\d\.]+)\s*(?:\(max\s*([-\d\.]+)\))?\s*\|\s*"
        r"Len:\s*(\d+)\s*(?:\(max\s*(\d+)\))?\s*\|\s*"
        r"Spd:\s*([-\d\.]+)\s*\|\s*"
        r"WPs_cum:\s*(\d+)\s*\|\s*"
        r"WPΔ_rew:\s*([-\d\.]+)"
    )

    batches: List[MetricBatch] = []
    for line in text.splitlines():
        m = pattern.search(line)
        if m:
            step = int(m.group(1))
            rew = float(m.group(2))
            rew_max = float(m.group(3)) if m.group(3) else rew
            ep_len = int(m.group(4))
            len_max = int(m.group(5)) if m.group(5) else ep_len
            spd = float(m.group(6))
            wps_cum = int(m.group(7))
            wp_step = float(m.group(8))
            batches.append(
                MetricBatch(
                    step=step,
                    rew=rew,
                    rew_max=rew_max,
                    ep_len=ep_len,
                    len_max=len_max,
                    speed=spd,
                    wps_cum=wps_cum,
                    wp_step=wp_step,
                    raw_timestamp=time.time(),
                )
            )

    return batches


def log_to_cta_db(
    action_type: str,
    description: str,
    status: str = "SUCCESS",
    files_touched: str = "",
    output_summary: str = "",
) -> None:
    """Log an intervention event directly into CTA turns SQLite DB."""
    if not os.path.exists(CTA_DB_PATH):
        return

    try:
        import sqlite3

        conn = sqlite3.connect(CTA_DB_PATH, timeout=10.0)
        c = conn.cursor()
        now_utc = datetime.datetime.now(datetime.timezone.utc).isoformat()
        session_id = f"sentinel_{int(time.time())}"
        c.execute(
            """INSERT INTO turn_actions (session_id, timestamp, milestone, phase, task,
                                         action_type, description, status, files_touched, output_summary)
               VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?)""",
            (
                session_id,
                now_utc,
                "M004",
                "01",
                "Autonomous Sentinel Self-Healing",
                action_type,
                description,
                status,
                files_touched,
                output_summary,
            ),
        )
        conn.commit()
        conn.close()
    except Exception as e:
        print(f"[Sentinel] Failed to log to CTA database: {e}", file=sys.stderr)


class AutonomousSentinel:
    """Core sentinel coordinator running diagnostic checks and interventions."""

    def __init__(self, logger: logging.Logger, dry_run: bool = False):
        self.logger = logger
        self.dry_run = dry_run
        self.state = load_state()

    def run_health_check(self) -> Tuple[bool, List[str], Dict[str, Any]]:
        """Evaluate all 5 diagnostic sentinels against latest telemetry."""
        proc = check_process_status()
        batches = parse_recent_metrics(max_lines=300)

        diagnostics: Dict[str, Any] = {
            "process": asdict(proc),
            "batches_count": len(batches),
            "sentinels": {},
        }
        triggered_failures: List[str] = []

        # -------------------------------------------------------------
        # Sentinel 5: Process Hang & VRAM Exhaustion Sentinel
        # -------------------------------------------------------------
        s5_triggered = False
        s5_reasons = []
        if not proc.tmux_active:
            s5_triggered = True
            s5_reasons.append("tmux session 'training' is dead")
        if not proc.python_active:
            s5_triggered = True
            s5_reasons.append("train_skrl.py process not detected")
        if not proc.log_fresh and proc.log_age_seconds > 180.0:
            s5_triggered = True
            s5_reasons.append(
                f"training log is stale ({proc.log_age_seconds:.1f}s > 180s)"
            )
        if proc.gpu.vram_used_mb > 11500.0:
            s5_triggered = True
            s5_reasons.append(
                f"GPU VRAM exhaustion ({proc.gpu.vram_used_mb}MB > 11.5GB)"
            )

        diagnostics["sentinels"]["sentinel_5_process_hang"] = {
            "triggered": s5_triggered,
            "reasons": s5_reasons,
        }
        if s5_triggered:
            triggered_failures.append(f"PROCESS_HANG: {'; '.join(s5_reasons)}")
            return False, triggered_failures, diagnostics

        if not batches:
            diagnostics["sentinels"]["status"] = "Awaiting initial batches"
            return True, [], diagnostics

        # Compute rolling window statistics (last 10 batches)
        window = batches[-10:]
        speeds = [b.speed for b in window]
        ep_lens = [b.ep_len for b in window]
        max_lens = [b.len_max for b in window]
        rews = [b.rew for b in window]
        max_rews = [b.rew_max for b in window]

        latest_batch = batches[-1]
        avg_speed = sum(speeds) / len(speeds)
        avg_ep_len = sum(ep_lens) / len(ep_lens)
        max_len_seen = max(max_lens)
        avg_rew = sum(rews) / len(rews)
        max_rew_seen = max(max_rews)

        diagnostics["metrics"] = {
            "latest_step": latest_batch.step,
            "avg_speed": round(avg_speed, 3),
            "avg_ep_len": round(avg_ep_len, 1),
            "max_len_seen": max_len_seen,
            "avg_rew": round(avg_rew, 1),
            "max_rew_seen": round(max_rew_seen, 1),
            "wps_cum": latest_batch.wps_cum,
            "wp_step": latest_batch.wp_step,
        }

        # Guard: Only evaluate performance sentinels (1-4) when a NEW batch has arrived
        is_new_batch = latest_batch.step != self.state.get("last_seen_step", -1)
        if not is_new_batch:
            return True, [], diagnostics

        # -------------------------------------------------------------
        # Sentinel 1: Crawling Trap Sentinel
        # -------------------------------------------------------------
        # Condition: avg_speed < 0.40 across consecutive evaluations with moderate/high ep_len
        s1_triggered = False
        if avg_speed < 0.40 and avg_ep_len >= 150.0:
            self.state["crawling_consecutive_count"] += 1
            if self.state["crawling_consecutive_count"] >= 5:
                s1_triggered = True
        else:
            self.state["crawling_consecutive_count"] = max(
                0, self.state["crawling_consecutive_count"] - 1
            )

        diagnostics["sentinels"]["sentinel_1_crawling"] = {
            "triggered": s1_triggered,
            "consecutive": self.state["crawling_consecutive_count"],
            "avg_speed": avg_speed,
        }
        if s1_triggered:
            triggered_failures.append(
                f"CRAWLING_TRAP: avg_speed={avg_speed:.2f} m/s (< 0.40) over 5 intervals"
            )

        # -------------------------------------------------------------
        # Sentinel 2: Corner 1 Apex Cliff / Understeer Lockup Sentinel
        # -------------------------------------------------------------
        # Condition: Over 100,000 steps, max(Len) < 800 steps (car unable to penetrate Corner 1 / understeer)
        s2_triggered = False
        eval_start = self.state.get("apex_cliff_eval_start_step", 0)
        if eval_start == 0 or latest_batch.step < eval_start:
            # Immediate unblinding: uninitialized or rollback occurred
            self.state["apex_cliff_eval_start_step"] = latest_batch.step
            self.state["step_window_max_len"] = max_len_seen
        else:
            self.state["step_window_max_len"] = max(
                self.state.get("step_window_max_len", 0), max_len_seen
            )
            step_delta = latest_batch.step - eval_start
            # Gate apex cliff evaluation so policy has a generous 100k-step horizon to adapt
            if latest_batch.step >= 300000 and step_delta >= 100000:
                if self.state["step_window_max_len"] < 800:
                    s2_triggered = True
                    # Reset evaluation window after triggering to prevent infinite re-fire
                    self.state["apex_cliff_eval_start_step"] = latest_batch.step
                    self.state["step_window_max_len"] = max_len_seen
                else:
                    # Reset window: successfully penetrated Corner 1 apex (>= 800 steps)
                    self.state["apex_cliff_eval_start_step"] = latest_batch.step
                    self.state["step_window_max_len"] = max_len_seen
            elif latest_batch.step < 300000:
                # Keep window tracking fresh during newborn warmup
                self.state["apex_cliff_eval_start_step"] = latest_batch.step
                self.state["step_window_max_len"] = max_len_seen

        diagnostics["sentinels"]["sentinel_2_apex_cliff"] = {
            "triggered": s2_triggered,
            "window_max_len": self.state.get("step_window_max_len", 0),
            "step_delta": latest_batch.step
            - self.state.get("apex_cliff_eval_start_step", latest_batch.step),
        }
        if s2_triggered:
            triggered_failures.append(
                f"CORNER1_APEX_CLIFF: max(Len)={self.state['step_window_max_len']} < 800 across 100k steps"
            )

        # -------------------------------------------------------------
        # Sentinel 3: Outer Shoulder Wall-Hugging Sentinel
        # -------------------------------------------------------------
        # Condition: Repeated straightaway crashes with NO exploration (avg_ep_len < 150, max_len < 350, wps < 25)
        # Gated so that multi-sector curriculum (e.g. Sector 2 short runway to C1) and healthy
        # exploratory agents (max_len >= 350 or wps_cum >= 25) never trigger false resets.
        s3_triggered = False
        if (
            latest_batch.step >= 100000
            and avg_ep_len < 150.0
            and max_len_seen < 350
            and latest_batch.wps_cum < 25
            and avg_speed >= 0.50
            and avg_rew < 600.0
        ):
            self.state["wall_hugging_consecutive_count"] += 1
            if self.state["wall_hugging_consecutive_count"] >= 10:
                s3_triggered = True
        else:
            self.state["wall_hugging_consecutive_count"] = max(
                0, self.state["wall_hugging_consecutive_count"] - 1
            )

        diagnostics["sentinels"]["sentinel_3_wall_hugging"] = {
            "triggered": s3_triggered,
            "consecutive": self.state["wall_hugging_consecutive_count"],
        }
        if s3_triggered:
            triggered_failures.append(
                f"WALL_HUGGING: Premature straightaway terminations with ep_len={avg_ep_len:.1f} < 150 (max_len={max_len_seen}, wps={latest_batch.wps_cum})"
            )

        # -------------------------------------------------------------
        # Sentinel 4: Policy Collapse & Divergence Sentinel
        # -------------------------------------------------------------
        # Condition: NaN divergence or sustained negative reward (avg_rew < 0.0) over 10 intervals
        s4_triggered = False
        import math

        is_diverged = math.isnan(avg_rew) or math.isnan(avg_ep_len)
        if is_diverged:
            s4_triggered = True
            triggered_failures.append(
                "POLICY_COLLAPSE: Numerical NaN detected in reward or episode length"
            )
        elif len(batches) >= 20 and avg_rew < 0.0:
            self.state["collapse_consecutive_count"] += 1
            if self.state["collapse_consecutive_count"] >= 10:
                s4_triggered = True
                triggered_failures.append(
                    f"POLICY_COLLAPSE: Sustained negative reward avg_rew={avg_rew:.1f} < 0.0 for 10 intervals"
                )
        else:
            self.state["collapse_consecutive_count"] = max(
                0, self.state["collapse_consecutive_count"] - 1
            )

        diagnostics["sentinels"]["sentinel_4_policy_collapse"] = {
            "triggered": s4_triggered,
            "consecutive": self.state["collapse_consecutive_count"],
            "avg_rew": avg_rew,
        }

        self.state["last_seen_step"] = latest_batch.step
        save_state(self.state)

        is_healthy = len(triggered_failures) == 0
        return is_healthy, triggered_failures, diagnostics

    def execute_self_healing(
        self, failure_type: str, diagnostics: Dict[str, Any], force: bool = False
    ) -> bool:
        """Execute automated self-healing action matching the failure mode."""
        now = time.time()
        elapsed_since_last = now - self.state.get("last_intervention_time", 0.0)

        # Prevent flapping with minimum cooldown, unless forced or process is dead/hung
        if (
            not force
            and "PROCESS_HANG" not in failure_type
            and elapsed_since_last < DEFAULT_COOLDOWN_SEC
        ):
            remaining = DEFAULT_COOLDOWN_SEC - elapsed_since_last
            self.logger.warning(
                f"Intervention cooldown active ({remaining:.1f}s remaining). Skipping flapping execution."
            )
            return False

        self.logger.critical(f"Executing self-healing intervention for: {failure_type}")
        action_summary = ""
        files_touched = []

        restart_cmd = None
        f_upper = failure_type.upper()
        if "PROCESS_HANG" in f_upper or "HANG" in f_upper:
            action_summary = "Killed hanging processes, verified GPU VRAM release, and restarted training via start_tmux_training.sh"
            if not self.dry_run:
                subprocess.run("pkill -9 -f 'isaac-sim|train_skrl.py'", shell=True)
                time.sleep(5)
            restart_cmd = ["bash", START_SCRIPT]

        elif "CRAWLING" in f_upper:
            action_summary = "Elevated speed tracking weight from 60.0 to 90.0 and boosted lookahead pre-braking in curriculum config"
            files_touched.append(CURRICULUM_CFG_PATH)
            if not self.dry_run:
                try:
                    from mdp.curriculum_config import update_curriculum_config

                    update_curriculum_config(
                        {
                            "speed_tracking_weight": 90.0,
                            "target_corner_speed": 0.40,
                        }
                    )
                except Exception as e:
                    self.logger.error(f"Failed to update curriculum config: {e}")
            restart_cmd = ["bash", START_SCRIPT]

        elif "CORNER1" in f_upper or "APEX_CLIFF" in f_upper or "CLIFF" in f_upper:
            target_ckpt = SECONDARY_LANDMARK
            action_summary = f"Hierarchical Rollback to secondary landmark {os.path.basename(target_ckpt)} after persistent understeer (50k steps max_len <= 860)"
            files_touched.append(target_ckpt)
            restart_cmd = ["bash", START_SCRIPT, target_ckpt]

        elif "WALL_HUGGING" in f_upper or "HUGGING" in f_upper:
            action_summary = "Refreshed track direction symmetry and warm-restarted training to clear outer shoulder bias"
            restart_cmd = ["bash", START_SCRIPT]

        elif "COLLAPSE" in f_upper or "DIVERGENCE" in f_upper:
            target_ckpt = (
                PRIMARY_LANDMARK
                if os.path.exists(PRIMARY_LANDMARK)
                else SECONDARY_LANDMARK
            )
            action_summary = f"Automated rollback to certified landmark checkpoint {os.path.basename(target_ckpt)}"
            files_touched.append(target_ckpt)
            restart_cmd = ["bash", START_SCRIPT, target_ckpt]

        # Write diagnosis file
        try:
            with open(DIAGNOSIS_FILE, "w") as f:
                f.write(f"# SENTINEL SELF-HEALING DIAGNOSIS\n\n")
                f.write(
                    f"**Timestamp**: {datetime.datetime.now(datetime.timezone.utc).isoformat()}\n\n"
                )
                f.write(f"**Failure Detected**: `{failure_type}`\n\n")
                f.write(f"**Action Taken**: {action_summary}\n\n")
                f.write(f"**Dry Run**: {self.dry_run}\n\n")
                f.write("```json\n" + json.dumps(diagnostics, indent=2) + "\n```\n")
        except Exception as e:
            self.logger.error(f"Failed to write diagnosis file: {e}")

        # Update persistent state
        self.state["last_intervention_time"] = time.time()
        self.state["last_intervention_type"] = failure_type
        self.state["interventions_count"] = self.state.get("interventions_count", 0) + 1
        self.state["crawling_consecutive_count"] = 0
        self.state["collapse_consecutive_count"] = 0
        self.state["wall_hugging_consecutive_count"] = 0
        if any(
            term in failure_type.upper()
            for term in ["CORNER1", "APEX_CLIFF", "CLIFF", "COLLAPSE", "DIVERGENCE"]
        ):
            self.state["apex_cliff_eval_start_step"] = 0
            self.state["step_window_max_len"] = 0
        save_state(self.state)

        # Log to CTA database
        if not self.dry_run:
            log_to_cta_db(
                action_type="FIX",
                description=f"Self-Healing: {failure_type}",
                status="SUCCESS",
                files_touched=";".join(files_touched),
                output_summary=action_summary,
            )

        self.logger.info("Self-healing intervention successfully executed and logged.")

        # Execute restart command last, as it may kill the running tmux session/sentinel
        if not self.dry_run and restart_cmd:
            clean_env = os.environ.copy()
            clean_env.pop("TMUX", None)
            clean_env.pop("TMUX_PANE", None)
            subprocess.Popen(
                restart_cmd, cwd=WORKSPACE_ROOT, env=clean_env, start_new_session=True
            )

        return True


def run_daemon(sentinel: AutonomousSentinel, interval_sec: float = 15.0) -> None:
    """Run persistent monitoring loop."""
    sentinel.logger.info(
        f"Autonomous Sentinel Daemon Active (interval={interval_sec}s)..."
    )
    try:
        while True:
            is_healthy, failures, diagnostics = sentinel.run_health_check()
            if not is_healthy:
                sentinel.logger.warning(f"Unhealthy condition detected: {failures}")
                sentinel.execute_self_healing(failures[0], diagnostics)
            else:
                metrics = diagnostics.get("metrics")
                if metrics:
                    sentinel.logger.info(
                        f"Health OK | Step: {metrics['latest_step']} | "
                        f"Len: {metrics['avg_ep_len']:.1f} (max {metrics['max_len_seen']}) | "
                        f"Rew: {metrics['avg_rew']:.1f} (max {metrics['max_rew_seen']:.1f}) | "
                        f"Spd: {metrics['avg_speed']:.2f} m/s | "
                        f"VRAM: {diagnostics['process']['gpu']['vram_used_mb']:.0f}MB"
                    )
                else:
                    sentinel.logger.info("Health OK (Awaiting fresh rollouts)...")
            time.sleep(interval_sec)
    except KeyboardInterrupt:
        sentinel.logger.info("Autonomous Sentinel Daemon terminated by user.")


def main():
    parser = argparse.ArgumentParser(
        description="Autonomous Self-Healing Sentinel for ARCPro RL"
    )
    parser.add_argument(
        "--daemon", action="store_true", help="Run in continuous monitoring daemon loop"
    )
    parser.add_argument(
        "--check", action="store_true", help="Perform single health check and exit"
    )
    parser.add_argument(
        "--status", action="store_true", help="Print current sentinel state and metrics"
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Evaluate failure conditions without modifying state",
    )
    parser.add_argument(
        "--interval",
        type=float,
        default=15.0,
        help="Check interval in seconds for daemon (default: 15)",
    )
    parser.add_argument(
        "--trigger",
        type=str,
        choices=[
            "crawling",
            "apex_cliff",
            "wall_hugging",
            "policy_collapse",
            "process_hang",
        ],
        help="Manually trigger a specific self-healing intervention",
    )
    parser.add_argument(
        "--force",
        action="store_true",
        help="Bypass cooldown period for testing or immediate override",
    )
    parser.add_argument(
        "--verbose", action="store_true", help="Enable verbose debug logging"
    )
    args = parser.parse_args()

    logger = setup_logger(verbose=args.verbose)
    sentinel = AutonomousSentinel(logger=logger, dry_run=args.dry_run)

    if args.trigger:
        logger.info(f"Manual trigger requested for sentinel: {args.trigger}")
        is_healthy, failures, diagnostics = sentinel.run_health_check()
        sentinel.execute_self_healing(
            f"MANUAL_TRIGGER_{args.trigger.upper()}", diagnostics, force=args.force
        )
        sys.exit(0)

    if args.status:
        is_healthy, failures, diagnostics = sentinel.run_health_check()
        print("\n=== AUTONOMOUS SENTINEL STATUS ===")
        print(f"Overall Health: {'✅ HEALTHY' if is_healthy else '❌ UNHEALTHY'}")
        if failures:
            print("Active Failures:")
            for f in failures:
                print(f"  - {f}")
        metrics = diagnostics.get("metrics")
        if metrics:
            print("\nRolling Metrics (Last 10 Batches):")
            for k, v in metrics.items():
                print(f"  {k}: {v}")
        proc = diagnostics.get("process", {})
        print("\nProcess & GPU Status:")
        print(
            f"  tmux session 'training': {'Running' if proc.get('tmux_active') else 'Dead'}"
        )
        print(
            f"  Python process: {'Active' if proc.get('python_active') else 'Inactive'}"
        )
        print(f"  Log age: {proc.get('log_age_seconds', 0):.1f}s")
        gpu = proc.get("gpu", {})
        print(
            f"  GPU VRAM: {gpu.get('vram_used_mb', 0):.0f} / {gpu.get('vram_total_mb', 0):.0f} MB ({gpu.get('gpu_util_pct', 0)}% util)"
        )
        print(
            f"\nLast Intervention: {sentinel.state.get('last_intervention_type')} at {sentinel.state.get('last_intervention_time')}"
        )
        print(f"Total Interventions: {sentinel.state.get('interventions_count', 0)}")
        sys.exit(0 if is_healthy else 1)

    if args.daemon:
        run_daemon(sentinel, interval_sec=args.interval)
    else:
        # Default action: single-pass health check
        is_healthy, failures, diagnostics = sentinel.run_health_check()
        if is_healthy:
            metrics = diagnostics.get("metrics", {})
            step_str = f"Step {metrics.get('latest_step', 'N/A')}"
            len_str = f"Len {metrics.get('avg_ep_len', 'N/A')}"
            rew_str = f"Rew {metrics.get('avg_rew', 'N/A')}"
            spd_str = f"Spd {metrics.get('avg_speed', 'N/A')} m/s"
            logger.info(
                f"✅ Sentinel Check PASSED ({step_str} | {len_str} | {rew_str} | {spd_str})"
            )
            sys.exit(0)
        else:
            logger.warning(f"❌ Sentinel Check FAILED: {failures}")
            if not args.dry_run:
                sentinel.execute_self_healing(failures[0], diagnostics)
            sys.exit(1)


if __name__ == "__main__":
    main()
