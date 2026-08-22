# Current State
**Status:** Training RGB-Only RL Agent (IsaacLab + SKRL)
**Phase:** Phase 1 (End-to-End Visual Driving with ResNet-18)
**Last Updated:** 2026-08-22

## Key Milestones & Breakthrough Results
1. **Observation Feedback & Actor Bounding (D023):**
   - Clamped `obs[:, 5:7]` in `observations.py` to `[-1.0, 1.0]`.
   - Bound actor mean with `torch.tanh` in `skrl_models.py`.
2. **Action Drive & Stagnation Fix (D024 / Issue 86):**
   - Restored `action_drive_reward = 20.0` in `arcpro_env_cfg.py` to eliminate crawling exploit.
   - Tightened `stagnation_termination` to 100 steps (2s) in `terminations.py` to eliminate survival farming.
3. **Dense Centerline Guidance Fix (D025 / Issue 87):**
   - Enabled `lateral_error = 1.0` in `arcpro_env_cfg.py` to provide smooth spatial gradients toward centerline before white line contact.
4. **Straightaway Stability Tuning (D026 / Issue 88):**
   - Increased `lateral_error` to `2.5` and enabled `jerk_penalty` at `0.05` in `arcpro_env_cfg.py`.
   - Completely resolved high-speed straightaway micro-weaving and lane-edge hugging.
5. **Record Performance Telemetry (>1.2M Steps):**
   - **Episode Length (`Len`):** Consistently peaking at **`705 steps`** (~14.1s continuous driving).
   - **Cumulative Waypoints (`WPs_cum`):** Reached **`1,457 Waypoints`** (~14.6m continuous track navigation).
   - **Speed (`Spd`):** Cruising at **`0.75 – 0.86 m/s`**.
   - **Episode Reward (`Rew`):** Shattered records up to **`+2,218.2`**.
6. **Codebase Quality & Disk Optimization (2026-08-22):**
   - Reclaimed **68+ GB** of disk space by pruning 24,000+ redundant intermediate 50-step checkpoints (retaining `best_agent.pt` and 100k milestones).
   - Cleaned legacy USD models in `openStreetUSD/`.
   - Cleaned dead MDP code (`observations_with_lat_err.py`) and organized root tests into `tests/`.
   - Increased `checkpoint_interval` in `train_skrl.py` to 500 rollouts to prevent future disk bloat.

## Current Focus
- Training is actively running in tmux session `training` (target 5,000,000 steps).
- Autonomous background watchdog is active on a 6-hour interval (`0 */6 * * *`).
- Context window ready for `/clear`.
