# Current State
**Status:** Training RGB-Only RL Agent (IsaacLab + SKRL)
**Phase:** Phase 1 (End-to-End Visual Driving with ResNet-18)
**Last Updated:** 2026-08-17

## Key Milestones & Breakthrough Results
1. **Observation Feedback & Actor Bounding (D023):**
   - Clamped `obs[:, 5:7]` in `observations.py` to `[-1.0, 1.0]`.
   - Bound actor mean with `torch.tanh` in `skrl_models.py`.
2. **Action Drive & Stagnation Fix (D024 / Issue 86):**
   - Restored `action_drive_reward = 20.0` in `arcpro_env_cfg.py` to eliminate crawling exploit.
   - Tightened `stagnation_termination` to 100 steps (2s) in `terminations.py` to eliminate survival farming.
3. **Dense Centerline Guidance Fix (D025 / Issue 87):**
   - Enabled `lateral_error = 1.0` (`-(abs_lat * 10.0)`) in `arcpro_env_cfg.py` to provide smooth spatial gradients toward centerline before white line contact.
4. **Massive Performance Surge at Step 401.6k:**
   - **Episode Reward (`Rew`):** Flipped strongly positive up to **`+949.6`**.
   - **Cumulative Waypoints (`WPs_cum`):** Reached **`706 Waypoints`** (>4.2m of clean track navigation).
   - **Speed (`Spd`):** Cruising at **`0.65 – 0.82 m/s`**.
   - **Episode Length (`Len`):** Extended to **`408 steps`** (~8.2 seconds continuous driving).
   - **Simulation Health:** 100% stable, no crashes, no memory leaks.

## Current Focus
- Training is actively running in tmux session `training` (target 5,000,000 steps).
- Autonomous background watchdog is active on a 6-hour interval (`0 */6 * * *`).
- Safe to clear context.
