# RESUME HERE — Project State & Context

## Project Status
- **Current Milestone:** Phase 1 (RGB-Only Visual Self-Driving with SKRL & ResNet-18)
- **Active Training:** Resumed in tmux session `training` from checkpoint `agent_906350.pt` (continuing toward 5,000,000 steps).
- **Peak Performance:** Step 881.4k crossed target with **`Rew: +1893.1`**, **`Len: 619 steps`**, **`Spd: 0.74 m/s`**, **`WPs: 888 waypoints`**.
- **Saved Best Checkpoint:** `logs/ppo_skrl/20260817-030056/26-08-17_03-00-56-420019_TelemetryPPO/checkpoints/best_agent.pt`

## Active Commands & Daemons
- **Training Session:** `tmux attach -t training`
- **Training Log:** `tail -f logs/skrl_phase1.log`
- **Autonomous Watchdog:** Running on a 6-hour cron (`0 */6 * * *`, task-432) with pre-approved auto-healing.
- **GUI Evaluation:**
  - View peak model: `bash debug.sh --checkpoint 881400` (or `best_agent.pt`)
  - View latest model: `bash debug.sh`

## Recent Critical Architecture & Reward Fixes
- **D023 (Architecture):** Clamped observation feedback `obs[:, 5:7]` to `[-1.0, 1.0]` and added `tanh` bounding to `ARCProActor` policy head.
- **D024 (Issue 86):** Restored `action_drive_reward = 20.0` and tightened `stagnation_termination` to 100 steps (2s) to destroy the creeping survival trap.
- **D025 (Issue 87):** Restored dense `lateral_error = 1.0` (`-(abs_lat * 10.0)`) to guide visual steering toward centerline waypoints well before white line contact.

## Next Steps for Incoming Agent
1. **Monitor Training Progression:** Allow training to run toward 5M steps. Check telemetry via `tail -n 25 logs/skrl_phase1.log`.
2. **GUI Checkpoint Inspection:** When requested by user or inspecting progress, run `bash debug.sh --checkpoint <step>`.
3. **Follow Global Rules:** Silently read `.planning/RESUME.md`, adhere to the Explicit Approval Rule, and keep summary at the end.
