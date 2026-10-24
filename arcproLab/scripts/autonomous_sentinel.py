#!/usr/bin/env python3
"""Autonomous Self-Healing Sentinel for ARCPro Reinforcement Learning."""

import argparse
import datetime
import json
import logging
import math
import os
import subprocess
import sys
import time
from dataclasses import asdict
from pathlib import Path
from typing import Any, Dict, List, Tuple

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
ARCPRO_LAB_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, ".."))
for _p in (SCRIPT_DIR, ARCPRO_LAB_DIR):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import sentinel_utils as su  # noqa: E402
from sentinel_utils import (  # noqa: E402
    START_SCRIPT,
    WORKSPACE_ROOT,
    load_state,
    save_state,
)


def __getattr__(name: str) -> Any:
    """Re-export all sentinel_utils symbols for backwards compatibility."""
    return getattr(su, name)


class AutonomousSentinel:
    """Core sentinel coordinator running diagnostic checks and interventions."""

    def __init__(self, logger: logging.Logger, dry_run: bool = False):
        self.logger = logger
        self.dry_run = dry_run
        self.state = load_state()

    def _apply_curriculum_stage(self, updates: Dict[str, Any]) -> None:
        """Apply curriculum weight/speed updates if not in dry-run mode."""
        if self.dry_run:
            return
        try:
            from mdp.curriculum_config import update_curriculum_config

            update_curriculum_config(updates)
        except Exception as e:
            self.logger.error(f"Failed to update curriculum config: {e}")

    def _spawn_detached_training(self, cmd: List[str], wait_s: float = 0.0) -> None:
        """Spawn start_tmux_training.sh in a detached session."""
        if self.dry_run or not cmd:
            return
        env = {k: v for k, v in os.environ.items() if k != "TMUX"}
        env["RUNNING_DETACHED"] = "1"
        self.logger.info(f"Spawning detached restart process: {' '.join(cmd)}")
        dn = subprocess.DEVNULL
        kw = dict(
            cwd=WORKSPACE_ROOT, env=env, start_new_session=True, stdout=dn, stderr=dn
        )
        subprocess.Popen(cmd, **kw)
        if wait_s > 0.0:
            time.sleep(wait_s)

    def run_health_check(self) -> Tuple[bool, List[str], Dict[str, Any]]:
        """Evaluate all 5 diagnostic sentinels against latest telemetry."""
        proc = su.check_process_status()
        batches = su.parse_recent_metrics(max_lines=300)
        sents: Dict[str, Any] = {}
        n_b = len(batches)
        diagnostics = {
            "process": asdict(proc),
            "batches_count": n_b,
            "sentinels": sents,
        }
        failures: List[str] = []

        # Sentinel 5: Process Hang & VRAM Exhaustion Sentinel
        if load_state().get("is_auditing", False):
            msg = "Audit in progress: training intentionally suspended"
            sents["sentinel_5_process_hang"] = {"triggered": False, "reasons": [msg]}
            return True, [], diagnostics

        s5: List[str] = []
        if not proc.tmux_active:
            s5.append("tmux session 'training' is dead")
        if not proc.python_active:
            s5.append("train_skrl.py process not detected")
        if not proc.log_fresh and proc.log_age_seconds > 180.0:
            s5.append(f"training log is stale ({proc.log_age_seconds:.1f}s > 180s)")
        if proc.gpu.vram_used_mb > 11500.0:
            s5.append(f"GPU VRAM exhaustion ({proc.gpu.vram_used_mb}MB > 11.5GB)")

        sents["sentinel_5_process_hang"] = {"triggered": bool(s5), "reasons": s5}
        if s5:
            failures.append(f"PROCESS_HANG: {'; '.join(s5)}")
            return False, failures, diagnostics
        if not batches:
            sents["status"] = "Awaiting initial batches"
            return True, [], diagnostics

        win = batches[-10:]
        last = batches[-1]
        spd = sum(b.speed for b in win) / len(win)
        avg_len = sum(b.ep_len for b in win) / len(win)
        max_len = max(b.len_max for b in win)
        rew = sum(b.rew for b in win) / len(win)
        max_rew = max(b.rew_max for b in win)

        diagnostics["metrics"] = dict(
            latest_step=last.step,
            avg_speed=round(spd, 3),
            avg_ep_len=round(avg_len, 1),
            max_len_seen=max_len,
            avg_rew=round(rew, 1),
            max_rew_seen=round(max_rew, 1),
            wps_cum=last.wps_cum,
            wp_step=last.wp_step,
        )
        if last.step == self.state.get("last_seen_step", -1):
            return True, [], diagnostics

        # Sentinel 1: Crawling Trap Sentinel
        s1 = False
        if spd < 0.25 and avg_len >= 150.0:
            self.state["crawling_consecutive_count"] += 1
            s1 = self.state["crawling_consecutive_count"] >= 5
        else:
            c1 = self.state["crawling_consecutive_count"] - 1
            self.state["crawling_consecutive_count"] = max(0, c1)
        c1 = self.state["crawling_consecutive_count"]
        sents["sentinel_1_crawling"] = {
            "triggered": s1,
            "consecutive": c1,
            "avg_speed": spd,
        }
        if s1:
            failures.append(f"CRAWLING_TRAP: avg_speed={spd:.2f} m/s (< 0.25)")

        # Sentinel 2: Corner 1 Apex Cliff / Understeer Lockup Sentinel
        s2 = False
        start_step = self.state.get("apex_cliff_eval_start_step", 0)
        if start_step == 0 or last.step < 300000:
            self.state["apex_cliff_eval_start_step"] = last.step
            self.state["step_window_max_len"] = max_len
        else:
            prev_max = self.state.get("step_window_max_len", 0)
            self.state["step_window_max_len"] = max(prev_max, max_len)
            if (last.step - start_step) >= 50000:
                if self.state["step_window_max_len"] <= 300:
                    s2 = True
                else:
                    self.state["apex_cliff_trigger_count"] = 0
                self.state["apex_cliff_eval_start_step"] = last.step
                self.state["step_window_max_len"] = max_len

        w_max = self.state.get("step_window_max_len", 0)
        ds = last.step - self.state.get("apex_cliff_eval_start_step", last.step)
        sents["sentinel_2_apex_cliff"] = dict(
            triggered=s2, window_max_len=w_max, step_delta=ds
        )
        if s2:
            failures.append(f"CORNER1_APEX_CLIFF: max(Len)={w_max} <= 300 in 50k steps")

        # Sentinel 3: Outer Shoulder Wall-Hugging Sentinel
        s3 = False
        if (
            last.step >= 100000
            and avg_len < 110.0
            and max_len < 300
            and spd >= 0.50
            and rew < 600.0
        ):
            self.state["wall_hugging_consecutive_count"] += 1
            s3 = self.state["wall_hugging_consecutive_count"] >= 15
        else:
            c3 = self.state["wall_hugging_consecutive_count"] - 1
            self.state["wall_hugging_consecutive_count"] = max(0, c3)
        c3 = self.state["wall_hugging_consecutive_count"]
        sents["sentinel_3_wall_hugging"] = {"triggered": s3, "consecutive": c3}
        if s3:
            failures.append(f"WALL_HUGGING: ep_len={avg_len:.1f} < 110, max={max_len}")

        # Sentinel 4: Policy Collapse & Divergence Sentinel
        s4 = False
        if math.isnan(rew) or math.isnan(avg_len):
            s4 = True
            failures.append("POLICY_COLLAPSE: Numerical NaN detected in telemetry")
        elif n_b >= 20 and rew < -500.0 and avg_len < 120.0:
            self.state["collapse_consecutive_count"] += 1
            if self.state["collapse_consecutive_count"] >= 10:
                s4 = True
                failures.append(f"POLICY_COLLAPSE: rew={rew:.1f}, len={avg_len:.1f}")
        else:
            c4 = self.state["collapse_consecutive_count"] - 1
            self.state["collapse_consecutive_count"] = max(0, c4)

        c4 = self.state["collapse_consecutive_count"]
        sents["sentinel_4_policy_collapse"] = {
            "triggered": s4,
            "consecutive": c4,
            "avg_rew": rew,
        }
        self.state["last_seen_step"] = last.step
        save_state(self.state)
        return len(failures) == 0, failures, diagnostics

    def execute_self_healing(
        self, failure_type: str, diagnostics: Dict[str, Any], force: bool = False
    ) -> bool:
        """Execute automated self-healing action matching the failure mode."""
        elapsed = time.time() - self.state.get("last_intervention_time", 0.0)
        is_hang = "PROCESS_HANG" in failure_type
        if not force and not is_hang and elapsed < su.DEFAULT_COOLDOWN_SEC:
            rem = su.DEFAULT_COOLDOWN_SEC - elapsed
            self.logger.warning(f"Cooldown active ({rem:.1f}s left). Skipping.")
            return False

        self.logger.critical(f"Executing self-healing intervention for: {failure_type}")
        summary, touched, restart_cmd = "", [], []
        f_up = failure_type.upper()

        if "PROCESS_HANG" in f_up or "HANG" in f_up:
            summary = (
                "Killed hanging processes and restarted via start_tmux_training.sh"
            )
            if not self.dry_run:
                subprocess.run("pkill -9 -f 'isaac-sim|train_skrl.py'", shell=True)
                time.sleep(5)
            restart_cmd = ["bash", START_SCRIPT]
        elif "CRAWLING" in f_up:
            summary = "Elevated speed_tracking_weight=90.0 and target_corner_speed=0.40"
            touched.append(su.CURRICULUM_CFG_PATH)
            self._apply_curriculum_stage(
                {"speed_tracking_weight": 90.0, "target_corner_speed": 0.40}
            )
            restart_cmd = ["bash", START_SCRIPT]
        elif "CORNER1" in f_up or "APEX_CLIFF" in f_up or "CLIFF" in f_up:
            trigs = self.state.get("apex_cliff_trigger_count", 0) + 1
            self.state["apex_cliff_trigger_count"] = trigs
            if trigs == 1:
                summary = "Curriculum boost (Attempt 1): Sector 2 weight set to 70%"
                touched.append(su.CURRICULUM_CFG_PATH)
                w = [0.10, 0.70, 0.05, 0.10, 0.05]
                self._apply_curriculum_stage(
                    {"sector_weights": w, "target_corner_speed": 0.35}
                )
            else:
                summary = f"Persistent Apex Cliff (Attempt {trigs}): Warm-restarting"
                self.state["apex_cliff_trigger_count"] = 0
                restart_cmd = ["bash", START_SCRIPT]
        elif "WALL_HUGGING" in f_up or "HUGGING" in f_up:
            summary = "Wall-hugging detected: Warm-restarting training cleanly"
            restart_cmd = ["bash", START_SCRIPT]
        elif "COLLAPSE" in f_up or "DIVERGENCE" in f_up:
            summary = "Policy Collapse detected: Warm-restarting training cleanly"
            restart_cmd = ["bash", START_SCRIPT]

        try:
            ts = datetime.datetime.now(datetime.timezone.utc).isoformat()
            with open(su.DIAGNOSIS_FILE, "w") as f:
                f.write(
                    f"# SENTINEL SELF-HEALING DIAGNOSIS\n\n**Timestamp**: {ts}\n\n"
                    f"**Failure Detected**: `{failure_type}`\n\n"
                    f"**Action Taken**: {summary}\n\n**Dry Run**: {self.dry_run}\n\n"
                    f"```json\n{json.dumps(diagnostics, indent=2)}\n```\n"
                )
        except Exception as e:
            self.logger.error(f"Failed to write diagnosis file: {e}")

        cnt = self.state.get("interventions_count", 0) + 1
        self.state["last_intervention_time"] = time.time()
        self.state["last_intervention_type"] = failure_type
        self.state["interventions_count"] = cnt
        self.state["crawling_consecutive_count"] = 0
        self.state["collapse_consecutive_count"] = 0
        self.state["wall_hugging_consecutive_count"] = 0
        save_state(self.state)

        if not self.dry_run:
            desc = f"Self-Healing: {failure_type}"
            su.log_to_cta_db("FIX", desc, "SUCCESS", ";".join(touched), summary)
        self.logger.info("Self-healing intervention successfully executed and logged.")
        self._spawn_detached_training(restart_cmd)
        return True

    def audit_checkpoint(
        self, ckpt_path: str, auto_heal: bool = True, steps: int = 150
    ) -> Tuple[bool, Dict[str, Any]]:
        """Run headless regression audit across 5 sectors for a checkpoint."""
        if not os.path.isfile(ckpt_path):
            self.logger.error(f"Checkpoint not found for audit: {ckpt_path}")
            return False, {}

        ckpt_key = su.format_checkpoint_key(ckpt_path)
        self.logger.info(f"🎯 Initiating Checkpoint Regression Audit for: {ckpt_key}")
        self.state["is_auditing"] = True
        save_state(self.state)

        proc = su.check_process_status()
        if proc.python_active or proc.tmux_active:
            self.logger.info("Stopping active training session to free GPU VRAM...")
            try:
                cmd_stop = ["tmux", "send-keys", "-t", "training:0.0", "C-c"]
                subprocess.run(cmd_stop, capture_output=True)
                time.sleep(10)
            except Exception:
                pass
            subprocess.run(
                "pkill -9 -f 'train_skrl.py'", shell=True, capture_output=True
            )
            time.sleep(5)

        gs = su.query_gpu_stats()
        self.logger.info(f"VRAM free: {gs.vram_used_mb:.0f}/{gs.vram_total_mb:.0f} MB")

        auditor = f"{WORKSPACE_ROOT}/arcproLab/scripts/checkpoint_auditor.py"
        sh_path = "/home/arika/IsaacLab/isaaclab.sh"
        cmd = [sh_path, "-p", auditor, "--checkpoint", ckpt_path, "--steps", str(steps)]
        if auto_heal:
            cmd.append("--auto_heal")

        self.logger.info(f"Executing auditor: {' '.join(cmd)}")
        passed, audit_output = False, ""
        try:
            env = {**os.environ, "TERM": "xterm"}
            kw = dict(
                cwd=WORKSPACE_ROOT, env=env, capture_output=True, text=True, timeout=400
            )
            res = subprocess.run(cmd, **kw)
            audit_output = res.stdout + "\n" + res.stderr
            if res.stdout:
                self.logger.info(res.stdout)
            if res.stderr and res.returncode != 0:
                self.logger.warning(res.stderr)
            passed = res.returncode == 0
            if passed:
                self.logger.info(f"✅ Checkpoint {ckpt_key} PASSED all sector audits!")
            else:
                self.logger.warning(f"⚠️ Checkpoint {ckpt_key} FAILED sector audits.")
        except subprocess.TimeoutExpired:
            self.logger.error(f"Audit timed out after 400s for {ckpt_key}")
        except Exception as e:
            self.logger.error(f"Audit failed with exception: {e}")

        audited = self.state.setdefault("audited_checkpoints", [])
        if ckpt_key not in audited:
            audited.append(ckpt_key)
        self.state["is_auditing"] = False
        save_state(self.state)

        self._spawn_detached_training(["bash", START_SCRIPT, ckpt_path], wait_s=25.0)
        return passed, {"output": audit_output}

    def check_and_audit_new_checkpoints(self) -> bool:
        """Check for new unaudited agent_*.pt checkpoints on active git branch."""
        log_subdir = su.get_active_log_subdir()
        if not os.path.isdir(log_subdir):
            return False

        audited = set(self.state.get("audited_checkpoints", []))
        candidates: List[Tuple[float, str, str]] = []
        now = time.time()

        for root, _, files in os.walk(log_subdir):
            if "corrupted" in root:
                continue
            for f in files:
                if f.startswith("agent_") and f.endswith(".pt"):
                    ckpt_path = Path(root) / f
                    p_dir = ckpt_path.parent
                    run_dir = p_dir.parent if p_dir.name == "checkpoints" else p_dir
                    ckpt_key = f"{run_dir.name}/{ckpt_path.name}"
                    try:
                        mtime = ckpt_path.stat().st_mtime
                        if (now - mtime) >= 20.0 and ckpt_key not in audited:
                            candidates.append((mtime, str(ckpt_path), ckpt_key))
                    except Exception:
                        pass

        if not candidates:
            return False

        candidates.sort(key=lambda x: x[0], reverse=True)
        _, target_ckpt, target_key = candidates[0]
        self.logger.info(f"Found new unaudited checkpoint: {target_key}")

        audited_list = self.state.setdefault("audited_checkpoints", [])
        for _, _, old_key in candidates[1:]:
            if old_key not in audited_list:
                audited_list.append(old_key)
        save_state(self.state)
        self.audit_checkpoint(target_ckpt, auto_heal=True)
        return True


def run_daemon(sentinel: AutonomousSentinel, interval_sec: float = 15.0) -> None:
    """Run persistent monitoring loop."""
    sentinel.logger.info(f"Sentinel Daemon Active (interval={interval_sec}s)")
    cycle = 0
    try:
        while True:
            ok, failures, diag = sentinel.run_health_check()
            if not ok:
                sentinel.logger.warning(f"Unhealthy condition detected: {failures}")
                sentinel.execute_self_healing(failures[0], diag)
            elif diag.get("metrics"):
                m = diag["metrics"]
                vram = diag["process"]["gpu"]["vram_used_mb"]
                sentinel.logger.info(
                    f"Health OK | Step: {m['latest_step']} | "
                    f"Len: {m['avg_ep_len']:.1f} (max {m['max_len_seen']}) | "
                    f"Rew: {m['avg_rew']:.1f} | Spd: {m['avg_speed']:.2f} | VRAM: {vram:.0f}MB"
                )
            else:
                sentinel.logger.info("Health OK (Awaiting fresh rollouts)...")

            cycle += 1
            if cycle % 2 == 0:
                sentinel.check_and_audit_new_checkpoints()
            time.sleep(interval_sec)
    except KeyboardInterrupt:
        sentinel.logger.info("Autonomous Sentinel Daemon terminated by user.")


def main() -> None:
    p = argparse.ArgumentParser(description="ARCPro Autonomous Self-Healing Sentinel")
    p.add_argument("--daemon", action="store_true", help="Run daemon monitoring loop")
    p.add_argument("--check", action="store_true", help="Run single health check")
    p.add_argument("--status", action="store_true", help="Print sentinel status")
    p.add_argument("--dry-run", action="store_true", help="Evaluate without acting")
    p.add_argument("--interval", type=float, default=15.0, help="Daemon poll seconds")
    t_opts = (
        "crawling",
        "apex_cliff",
        "wall_hugging",
        "policy_collapse",
        "process_hang",
    )
    p.add_argument("--trigger", type=str, choices=t_opts, help="Manual trigger")
    p.add_argument("--force", action="store_true", help="Bypass intervention cooldown")
    p.add_argument("--audit", type=str, default="", help="Audit checkpoint path")
    p.add_argument("--audit-latest", action="store_true", help="Audit latest ckpt")
    p.add_argument("--verbose", action="store_true", help="Enable debug logging")
    args = p.parse_args()

    logger = su.setup_logger(verbose=args.verbose)
    sentinel = AutonomousSentinel(logger=logger, dry_run=args.dry_run)

    if args.audit or args.audit_latest:
        target = args.audit or su.get_current_run_checkpoint()
        if not target:
            logger.error("No checkpoint found to audit.")
            sys.exit(1)
        logger.info(f"Manual regression audit requested for checkpoint: {target}")
        passed, _ = sentinel.audit_checkpoint(target, auto_heal=True)
        sys.exit(0 if passed else 1)
    if args.trigger:
        logger.info(f"Manual trigger requested for sentinel: {args.trigger}")
        _, _, diag = sentinel.run_health_check()
        tag = f"MANUAL_TRIGGER_{args.trigger.upper()}"
        sentinel.execute_self_healing(tag, diag, force=args.force)
        sys.exit(0)
    if args.status:
        sys.exit(su.print_status(sentinel))
    if args.daemon:
        run_daemon(sentinel, interval_sec=args.interval)
    else:
        ok, failures, diag = sentinel.run_health_check()
        if ok:
            m = diag.get("metrics", {})
            logger.info(
                f"✅ Sentinel Check PASSED (Step {m.get('latest_step', 'N/A')} | "
                f"Len {m.get('avg_ep_len', 'N/A')} | Rew {m.get('avg_rew', 'N/A')} | "
                f"Spd {m.get('avg_speed', 'N/A')} m/s)"
            )
            sys.exit(0)
        logger.warning(f"❌ Sentinel Check FAILED: {failures}")
        if not args.dry_run:
            sentinel.execute_self_healing(failures[0], diag)
        sys.exit(1)


if __name__ == "__main__":
    main()
