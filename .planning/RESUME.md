# RESUME — Context for Next Agent Session
**Last Updated:** 2026-08-22

---

## 1. Executive Summary & Breakthrough Milestones

### Peak Telemetry Achieved (>1,217,700 Steps)
- **Episode Reward (`Rew`):** **`+2,218.2`**!
- **Cumulative Waypoints (`WPs_cum`):** **`1,457 Waypoints`** (over 14.5 meters of continuous track navigation).
- **Speed (`Spd`):** Cruising at **`0.75 – 0.86 m/s`** (exceeding strict success criteria).
- **Episode Length (`Len`):** Consistently peaking at **`705 steps`** (~14.1 seconds continuous driving).
- **Per-Step WP Reward Delta (`WPΔ_rew`):** Firing strongly at **`4.38 – 5.22`** per step.
- **Saved Best Model:** [`logs/ppo_skrl/20260819-214341/26-08-19_21-43-41-085534_TelemetryPPO/checkpoints/best_agent.pt`](file:///home/arika/Documents/arcpro/arcpro_system/src/examples/ARCPro_RL/arc_rl_isacc_sim/logs/ppo_skrl/20260819-214341/26-08-19_21-43-41-085534_TelemetryPPO/checkpoints/best_agent.pt)

---

## 2. Root Cause Fixes Applied in This Project

### Issue 86: Crawling Survival Trap & Action Drive Restoration
- **Problem:** Agent coasted at `0.04 - 0.06 m/s` for 250+ steps because `action_drive_reward` was 0.0 and `stagnation_termination` was too lenient (300 steps).
- **Fix:** Restored `action_drive_reward = 20.0` in [`arcpro_env_cfg.py`](file:///home/arika/Documents/arcpro/arcpro_system/src/examples/ARCPro_RL/arc_rl_isacc_sim/arcproLab/arcpro_env_cfg.py) and tightened `stagnation_termination` to 100 steps in [`terminations.py`](file:///home/arika/Documents/arcpro/arcpro_system/src/examples/ARCPro_RL/arc_rl_isacc_sim/arcproLab/mdp/terminations.py).

### Issue 87: Dense Centerline Guidance Restoration
- **Problem:** With `lateral_error = 0.0` and `boundary_penalty = 0.0`, the agent had no spatial gradient informing it of centerline drift until it slammed into a line and suffered a sudden -500 death penalty.
- **Fix:** Enabled `lateral_error = 1.0` in [`arcpro_env_cfg.py`](file:///home/arika/Documents/arcpro/arcpro_system/src/examples/ARCPro_RL/arc_rl_isacc_sim/arcproLab/arcpro_env_cfg.py).

### Issue 88: Straightaway Stability Tuning (D026)
- **Problem:** At 0.85 m/s, no jerk penalty allowed high-frequency micro-weaving, and weak lateral error allowed edge-hugging.
- **Fix:** Increased `lateral_error` to `2.5` and enabled `jerk_penalty = 0.05` in [`arcpro_env_cfg.py`](file:///home/arika/Documents/arcpro/arcpro_system/src/examples/ARCPro_RL/arc_rl_isacc_sim/arcproLab/arcpro_env_cfg.py).

---

## 3. Active Background Infrastructure

- **Training Session:** Actively running in tmux session `training` from [`agent_1217700.pt`](file:///home/arika/Documents/arcpro/arcpro_system/src/examples/ARCPro_RL/arc_rl_isacc_sim/logs/ppo_skrl/20260819-214341/26-08-19_21-43-41-085534_TelemetryPPO/checkpoints/agent_1217700.pt) (32 parallel environments, `--enable_cameras` active, continuing toward 5,000,000 steps).
- **Autonomous Watchdog:** Scheduled on a **6-hour interval** (`0 */6 * * *`, task-533).
- **Disk Space Clean:** Pruned 24,000+ intermediate checkpoints recovering 68 GB disk space; `checkpoint_interval` set to 500.

---

## 4. Next Steps for Incoming Agent

1. **Do NOT interrupt or tweak rewards prematurely.** The policy is training smoothly toward 5M steps to connect full track laps.
2. **Monitor Telemetry:** Check progress via `tail -n 25 logs/skrl_phase1.log` or `tmux attach -t training`.
3. **GUI Evaluation:** To inspect checkpoints in GUI, run:
   ```bash
   bash debug.sh --checkpoint best_agent.pt
   bash debug.sh                         # Latest checkpoint
   ```
4. **Strict Rules:** Always follow the GSD Auto-Resume Rule, Explicit Approval Rule, and Workspace Context Rule.

