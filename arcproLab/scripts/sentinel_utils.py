#!/usr/bin/env python3
"""Shared telemetry, process probe, checkpoint discovery, and state utilities."""

import datetime
import json
import logging
import os
import re
import sqlite3
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional

WORKSPACE_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "../.."))
LOG_FILE = f"{WORKSPACE_ROOT}/logs/skrl_phase1.log"
SENTINEL_LOG = f"{WORKSPACE_ROOT}/logs/sentinel.log"
DIAGNOSIS_FILE = f"{WORKSPACE_ROOT}/logs/SENTINEL_DIAGNOSIS.md"
STATE_FILE = f"{WORKSPACE_ROOT}/logs/sentinel_state.json"
CTA_DB_PATH = f"{WORKSPACE_ROOT}/.cta/cta_turns.db"
CURRICULUM_CFG_PATH = f"{WORKSPACE_ROOT}/arcproLab/mdp/curriculum_sectors.json"
START_SCRIPT = f"{WORKSPACE_ROOT}/start_tmux_training.sh"
DEFAULT_COOLDOWN_SEC = 300.0


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


@dataclass
class SelfHealingAction:
    action_type: str
    summary: str
    restart_cmd: Optional[List[str]] = None
    curriculum_updates: Optional[Dict[str, Any]] = None
    new_curriculum_stage: Optional[int] = None


def get_active_branch() -> str:
    """Detect active git branch, falling back cleanly if git is unavailable."""
    try:
        cmd = ["git", "rev-parse", "--abbrev-ref", "HEAD"]
        out = subprocess.check_output(cmd, cwd=WORKSPACE_ROOT, text=True, timeout=3)
        return out.strip()
    except Exception:
        return "rollback/clean-baseline"


def get_active_log_subdir(branch: Optional[str] = None) -> str:
    """Resolve the active SKRL log subdirectory for the active git branch."""
    tag = (branch or get_active_branch()).replace("/", "_")
    sub = f"{WORKSPACE_ROOT}/logs/ppo_skrl_{tag}"
    return sub if os.path.isdir(sub) else f"{WORKSPACE_ROOT}/logs/ppo_skrl"


def format_checkpoint_key(ckpt_path: str) -> str:
    """Return 'run_dir_name/ckpt_filename' to prevent cross-run name collisions."""
    p = Path(ckpt_path)
    run_dir = p.parent.parent if p.parent.name == "checkpoints" else p.parent
    return f"{run_dir.name}/{p.name}"


def get_current_run_checkpoint() -> str:
    """Find latest step checkpoint (or best_agent.pt fallback) in active run."""
    log_subdir = get_active_log_subdir()
    if not os.path.isdir(log_subdir):
        return ""
    best_ckpts, agent_ckpts = [], []
    for root, _, files in os.walk(log_subdir):
        if "corrupted" in root:
            continue
        for f in files:
            p = os.path.join(root, f)
            if f == "best_agent.pt":
                best_ckpts.append((os.path.getmtime(p), p))
            elif f.startswith("agent_") and f.endswith(".pt"):
                agent_ckpts.append((os.path.getmtime(p), p))
    if agent_ckpts:
        return sorted(agent_ckpts)[-1][1]
    return sorted(best_ckpts)[-1][1] if best_ckpts else ""


def setup_logger(verbose: bool = False) -> logging.Logger:
    """Setup dual console and file logging for Autonomous Sentinel."""
    lvl = logging.DEBUG if verbose else logging.INFO
    logger = logging.getLogger("autonomous_sentinel")
    logger.setLevel(lvl)
    logger.handlers.clear()
    os.makedirs(os.path.dirname(SENTINEL_LOG), exist_ok=True)
    fh = logging.FileHandler(SENTINEL_LOG)
    fh.setLevel(logging.DEBUG)
    fh.setFormatter(logging.Formatter("[%(asctime)s] [%(levelname)s] %(message)s"))
    logger.addHandler(fh)
    ch = logging.StreamHandler(sys.stdout)
    ch.setLevel(lvl)
    ch.setFormatter(logging.Formatter("[Sentinel] %(message)s"))
    logger.addHandler(ch)
    return logger


def load_state() -> Dict[str, Any]:
    """Load persistent sentinel state from JSON."""
    state: Dict[str, Any] = {
        "last_intervention_time": 0.0,
        "last_intervention_type": None,
        "interventions_count": 0,
        "crawling_consecutive_count": 0,
        "collapse_consecutive_count": 0,
        "wall_hugging_consecutive_count": 0,
        "last_seen_step": 0,
        "step_window_max_len": 0,
        "apex_cliff_eval_start_step": 0,
        "apex_cliff_trigger_count": 0,
        "audited_checkpoints": [],
    }
    if os.path.exists(STATE_FILE):
        try:
            with open(STATE_FILE, "r") as f:
                loaded = json.load(f)
                loaded.pop("step_window_start", None)
                state.update(loaded)
        except Exception:
            pass
    state.setdefault("audited_checkpoints", [])
    return state


def save_state(state: Dict[str, Any]) -> None:
    """Atomically save persistent sentinel state to JSON."""
    os.makedirs(os.path.dirname(STATE_FILE), exist_ok=True)
    tmp_path = STATE_FILE + ".tmp"
    with open(tmp_path, "w") as f:
        json.dump(state, f, indent=2)
    os.replace(tmp_path, STATE_FILE)


def query_gpu_stats() -> GPUStats:
    """Query nvidia-smi safely for GPU memory and utilization."""
    try:
        q = "--query-gpu=memory.used,memory.total,utilization.gpu"
        cmd = ["nvidia-smi", q, "--format=csv,nounits,noheader"]
        res = subprocess.check_output(cmd, timeout=3).decode("utf-8").strip()
        parts = [p.strip() for p in res.split(",")]
        if len(parts) >= 3:
            return GPUStats(float(parts[0]), float(parts[1]), float(parts[2]))
    except Exception:
        pass
    return GPUStats()


def check_process_status(max_stale_seconds: float = 180.0) -> ProcessStatus:
    """Check tmux session, process execution, and log freshness."""
    tmux_ok, py_ok = False, False
    try:
        cmd = ["tmux", "has-session", "-t", "training"]
        tmux_ok = subprocess.run(cmd, capture_output=True).returncode == 0
    except Exception:
        pass
    try:
        cmd = ["pgrep", "-f", "train_skrl.py"]
        py_ok = subprocess.run(cmd, capture_output=True).returncode == 0
    except Exception:
        pass
    log_exists = os.path.exists(LOG_FILE)
    log_age, log_fresh = 999999.0, False
    if log_exists:
        try:
            log_age = time.time() - os.path.getmtime(LOG_FILE)
            log_fresh = log_age <= max_stale_seconds
        except Exception:
            pass
    return ProcessStatus(
        tmux_ok, py_ok, log_exists, log_fresh, log_age, query_gpu_stats()
    )


_METRIC_PATTERN = re.compile(
    r"Step\s+(\d+)\s*\([^)]*\)\s*\|\s*Rew:\s*([-\d\.]+)\s*(?:\(max\s*([-\d\.]+)\))?\s*\|\s*"
    r"Len:\s*(\d+)\s*(?:\(max\s*(\d+)\))?\s*\|\s*Spd:\s*([-\d\.]+)\s*\|\s*WPs_cum:\s*(\d+)\s*\|\s*WPΔ_rew:\s*([-\d\.]+)"
)


def parse_recent_metrics(max_lines: int = 300) -> List[MetricBatch]:
    """Parse telemetry batches from the tail of skrl_phase1.log."""
    if not os.path.exists(LOG_FILE):
        return []
    try:
        raw = subprocess.check_output(
            ["tail", "-n", str(max_lines), LOG_FILE], timeout=5
        )
        text = raw.decode("utf-8", errors="replace")
    except Exception:
        return []
    batches: List[MetricBatch] = []
    now = time.time()
    for line in text.splitlines():
        m = _METRIC_PATTERN.search(line)
        if m:
            rew, ep_len = float(m.group(2)), int(m.group(4))
            rmax = float(m.group(3)) if m.group(3) else rew
            lmax = int(m.group(5)) if m.group(5) else ep_len
            batches.append(
                MetricBatch(
                    int(m.group(1)),
                    rew,
                    rmax,
                    ep_len,
                    lmax,
                    float(m.group(6)),
                    int(m.group(7)),
                    float(m.group(8)),
                    now,
                )
            )
    return batches


def log_to_cta_db(
    action_type: str,
    description: str,
    status: str = "SUCCESS",
    files_touched: str = "",
    output_summary: str = "",
    milestone: str = "M004",
    phase: str = "01",
    task: str = "Autonomous Sentinel Self-Healing",
) -> None:
    """Log an intervention or audit event directly into CTA turns SQLite DB."""
    if not os.path.exists(CTA_DB_PATH):
        return
    try:
        conn = sqlite3.connect(CTA_DB_PATH, timeout=10.0)
        now_utc = datetime.datetime.now(datetime.timezone.utc).isoformat()
        sql = (
            "INSERT INTO turn_actions (session_id, timestamp, milestone, phase, task, "
            "action_type, description, status, files_touched, output_summary) "
            "VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?)"
        )
        sid = f"sentinel_{int(time.time())}"
        row = (
            sid,
            now_utc,
            milestone,
            phase,
            task,
            action_type,
            description,
            status,
            files_touched,
            output_summary,
        )
        conn.execute(sql, row)
        conn.commit()
        conn.close()
    except Exception as e:
        print(f"[Sentinel] Failed to log to CTA database: {e}", file=sys.stderr)


def print_status(sentinel: Any) -> int:
    """Print human-readable status summary and return exit code."""
    is_healthy, failures, diagnostics = sentinel.run_health_check()
    health_str = "✅ HEALTHY" if is_healthy else "❌ UNHEALTHY"
    print(f"\n=== AUTONOMOUS SENTINEL STATUS ===\nOverall Health: {health_str}")
    for f in failures:
        print(f"  - {f}")
    if diagnostics.get("metrics"):
        print("\nRolling Metrics (Last 10 Batches):")
        for k, v in diagnostics["metrics"].items():
            print(f"  {k}: {v}")
    proc = diagnostics.get("process", {})
    gpu = proc.get("gpu", {})
    tmux_s = "Running" if proc.get("tmux_active") else "Dead"
    py_s = "Active" if proc.get("python_active") else "Inactive"
    v_used, v_tot = gpu.get("vram_used_mb", 0), gpu.get("vram_total_mb", 0)
    print(
        f"\nProcess & GPU Status:\n  tmux session 'training': {tmux_s}\n"
        f"  Python process: {py_s}\n  Log age: {proc.get('log_age_seconds', 0):.1f}s\n"
        f"  GPU VRAM: {v_used:.0f} / {v_tot:.0f} MB ({gpu.get('gpu_util_pct', 0)}% util)\n\n"
        f"Last Intervention: {sentinel.state.get('last_intervention_type')} "
        f"at {sentinel.state.get('last_intervention_time')}\n"
        f"Total Interventions: {sentinel.state.get('interventions_count', 0)}\n"
        f"Audited Checkpoints: {sentinel.state.get('audited_checkpoints', [])}"
    )
    return 0 if is_healthy else 1
