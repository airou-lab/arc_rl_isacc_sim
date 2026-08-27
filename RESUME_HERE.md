# RESUME HERE — Project State & Context

## Project Status
- **Current Milestone:** Phase 1 (RGB-Only Visual Self-Driving with SKRL & ResNet-18)
- **Active Training:** Resumed in tmux session `training` from checkpoint `agent_1217700.pt` (continuing toward 5,000,000 steps).
- **Peak Performance:** Crossed record with **`Rew: +2218.2`**, **`Len: 705 steps`** (~14.1s), **`Spd: 0.86 m/s`**, **`WPs: 1,457 waypoints`**.
- **Saved Best Checkpoint:** `logs/ppo_skrl/20260819-214341/26-08-19_21-43-41-085534_TelemetryPPO/checkpoints/best_agent.pt`

## Active Commands & Daemons
- **Training Session:** `tmux attach -t training`
- **Training Log:** `tail -f logs/skrl_phase1.log`
- **Autonomous Watchdog:** Running on a 6-hour cron (`0 */6 * * *`, task-533) with pre-approved auto-healing.
- **GUI Evaluation:**
  - View peak model: `bash debug.sh --checkpoint best_agent.pt`
  - View latest model: `bash debug.sh`

## Recent Critical Architecture & Reward Fixes
- **D023 (Architecture):** Clamped observation feedback `obs[:, 5:7]` to `[-1.0, 1.0]` and added `tanh` bounding to `ARCProActor` policy head.
- **D024 (Issue 86):** Restored `action_drive_reward = 20.0` and tightened `stagnation_termination` to 100 steps (2s) to destroy the creeping survival trap.
- **D025 (Issue 87):** Restored dense `lateral_error = 1.0` to guide visual steering toward centerline waypoints.
- **D026 (Issue 88):** Applied Straightaway Stability Tuning: Increased `lateral_error` to 2.5 and enabled `jerk_penalty` at 0.05 to prevent high-speed weaving.
- **D027 (Issue 89):** Fixed silent `prev_action` no-op bug in `rewards.py`, activated `jerk_penalty = 0.5` to eliminate bang-bang oscillation, set steering `offset = 0.0`, and boosted `lateral_error = 10.0` for solid centerline tracking.
- **Codebase Quality & Disk Cleanup (2026-08-22):** Pruned 24,000+ intermediate checkpoints (reclaimed 68 GB), cleaned legacy USDs, and moved root tests to `tests/`.

## Next Steps for Incoming Agent
1. **Monitor Training Progression:** Allow training to run toward 5M steps. Check telemetry via `tail -n 25 logs/skrl_phase1.log`.
2. **GUI Checkpoint Inspection:** When requested by user or inspecting progress, run `bash debug.sh --checkpoint <step>`.
3. **Follow Global Rules:** Silently read `.planning/RESUME.md`, adhere to the Explicit Approval Rule, and keep summary at the end.

