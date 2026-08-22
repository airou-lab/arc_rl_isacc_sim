# RESUME — Context for Next Agent Session
**Last Updated:** 2026-08-18

---

## 1. Executive Summary & Breakthrough Milestones

### Peak Telemetry Achieved (Step 881,400 / 5,000,000)
- **Episode Reward (`Rew`):** **`+1,893.1`**!
- **Cumulative Waypoints (`WPs_cum`):** **`888 Waypoints`** (over 5.3 meters of continuous track navigation).
- **Speed (`Spd`):** Cruising at **`0.74 – 0.83 m/s`** (exceeding strict success criteria).
- **Episode Length (`Len`):** **`619 steps`** (~12.4 seconds continuous driving — Success Criteria Met!).
- **Per-Step WP Reward Delta (`WPΔ_rew`):** Firing strongly at **`4.09 – 4.84`** per step.
- **Saved Best Model:** [`logs/ppo_skrl/20260817-030056/26-08-17_03-00-56-420019_TelemetryPPO/checkpoints/best_agent.pt`](file:///home/arika/Documents/arcpro/arcpro_system/src/examples/ARCPro_RL/arc_rl_isacc_sim/logs/ppo_skrl/20260817-030056/26-08-17_03-00-56-420019_TelemetryPPO/checkpoints/best_agent.pt)

---

## 2. Root Cause Fixes Applied in This Project

### Issue 86: Crawling Survival Trap & Action Drive Restoration
- **Problem:** Agent coasted at `0.04 - 0.06 m/s` for 250+ steps because `action_drive_reward` was 0.0 and `stagnation_termination` was too lenient (300 steps).
- **Fix:** Restored `action_drive_reward = 20.0` in [`arcpro_env_cfg.py`](file:///home/arika/Documents/arcpro/arcpro_system/src/examples/ARCPro_RL/arc_rl_isacc_sim/arcproLab/arcpro_env_cfg.py) and tightened `stagnation_termination` to 100 steps in [`terminations.py`](file:///home/arika/Documents/arcpro/arcpro_system/src/examples/ARCPro_RL/arc_rl_isacc_sim/arcproLab/mdp/terminations.py).

### Issue 87: Dense Centerline Guidance Restoration
- **Problem:** With `lateral_error = 0.0` and `boundary_penalty = 0.0`, the agent had no spatial gradient informing it of centerline drift until it slammed into a line and suffered a sudden -500 death penalty.
- **Fix:** Enabled `lateral_error = 1.0` in [`arcpro_env_cfg.py`](file:///home/arika/Documents/arcpro/arcpro_system/src/examples/ARCPro_RL/arc_rl_isacc_sim/arcproLab/arcpro_env_cfg.py), applying `-(abs_lat * 10.0)` dense continuous feedback.

---

## 3. Active Background Infrastructure

- **Training Session:** Actively resumed in tmux session `training` from [`agent_906350.pt`](file:///home/arika/Documents/arcpro/arcpro_system/src/examples/ARCPro_RL/arc_rl_isacc_sim/logs/ppo_skrl/20260817-030056/26-08-17_03-00-56-420019_TelemetryPPO/checkpoints/agent_906350.pt) (32 parallel environments, `--enable_cameras` active, continuing toward 5,000,000 steps).
- **Autonomous Watchdog:** Scheduled on a **6-hour interval** (`0 */6 * * *`, task-432).
  - Pre-approved by user to auto-diagnose, auto-fix, document in `.planning/reward_tuning_history.md`, wipe checkpoints, and restart training if stagnation or failure occurs.

---

## 4. Next Steps for Incoming Agent

1. **Do NOT interrupt or tweak rewards prematurely.** The policy is training through the remaining 4M steps to finish full track laps.
2. **Monitor Telemetry:** Check progress via `tail -n 25 logs/skrl_phase1.log` or `tmux attach -t training`.
3. **GUI Evaluation:** To inspect checkpoints in GUI, run:
   ```bash
   bash debug.sh --checkpoint 881400     # Peak model
   bash debug.sh --checkpoint best_agent.pt
   bash debug.sh                         # Latest checkpoint
   ```
4. **Strict Rules:** Always follow the GSD Auto-Resume Rule, Explicit Approval Rule, and Workspace Context Rule.
