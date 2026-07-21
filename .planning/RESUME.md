# ARCPro RL Training - Session Resume Context
**Date:** 2026-07-18 (Updated 17:50)

## Current Goal
**Achieve the Strict Target:** `ep_len_mean >= 600` AND `speed > 0.5 m/s` AND `WPΔ_rew > 0` (real waypoint progress via the reward signal, not the obs layer).

---

## Training Status
- **tmux session:** `training` — **ACTIVE** (PPO running, ~716k steps when last checked)
- **Log file:** `logs/skrl_phase1.log`
- **Branch:** `skrl` (committed 2026-07-18, hash `8d07584`)
- **Straight-line test:** ✅ PASSED — 500 steps, zero resets, max_speed = 1.968 m/s

> ⚠️ Training is still using the OLD physics (pre-fix). Needs a **restart** to pick up the 3 physics fixes below.

---

## Physics Fixes Applied This Session (2026-07-18) — Issue 15

| Fix | File | What Changed |
|-----|------|--------------|
| **15A — Tire Friction** | `mdp/spawner.py` | `_apply_tire_friction()`: binds `UsdPhysics.MaterialAPI` to all `Wheel_*` prims. `static=1.2`, `dynamic=0.8`, `restitution=0.0` |
| **15B — Chassis Damping** | `arcpro_robot_cfg.py` | `linear_damping 0.0→0.1`, `angular_damping 0.0→0.05` — rolling resistance + spin-out prevention |
| **15C — Spawn & Thresholds** | `arcpro_env_cfg.py` | Removed `lin_vel` kickstart. `height_termination` threshold `0.02→0.005m`. `stationary` speed threshold `0.5→0.2 m/s` |

### Why this matters
Before fixes: car died every 49–55 steps (height termination) due to kickstart bounce.
After fixes: straight-line test survived 500 steps cleanly. Z axis stable. Car now behaves like an RC car.

---

## Previous Fixes (SKRL Migration, 2026-07-17)

| Fix | File | What Changed |
|-----|------|--------------|
| **A** | `train_skrl.py` | Replaced `Trk: 0.0%` (permanently zero) with `WPs_cum` (raw WP count) |
| **B** | `train_skrl.py` + `rewards.py` | Log now shows `WPΔ_rew` — the actual per-step reward variable |
| **C** | `rewards.py` | **Displacement gate**: reward only fires if robot moved ≥2cm |
| **D** | `arcpro_env_cfg.py` | `progress_reward` weight `5.0 → 20.0` |
| **Bug 5** | `arcpro_env_cfg.py` | `stationary` lambda uses `torch.abs()` |
| **debug.sh** | `debug.sh` | Fixed: now calls `play_skrl.py` with auto-detection of latest checkpoint |

---

## Log Format
```
Step N | Rew: X | Len: Y | Spd: Z | WPs_cum: A | WPΔ_rew: B
```
- `WPΔ_rew > 0.5` consistently = agent is genuinely navigating
- `WPΔ_rew` spikes to 2000+ = Bug 16 race condition firing (see reward_tuning_history.md)

---

## Key Files
- [`arcproLab/mdp/spawner.py`](arcproLab/mdp/spawner.py) — `_apply_tire_friction()`: RC car tire physics
- [`arcproLab/arcpro_robot_cfg.py`](arcproLab/arcpro_robot_cfg.py) — chassis damping
- [`arcproLab/mdp/rewards.py`](arcproLab/mdp/rewards.py) — `waypoint_progress_reward()`: displacement gate + extras logging
- [`arcproLab/arcpro_env_cfg.py`](arcproLab/arcpro_env_cfg.py) — reward weights, terminations, spawn
- [`arcproLab/scripts/train_skrl.py`](arcproLab/scripts/train_skrl.py) — main trainer
- [`arcproLab/scripts/play_skrl.py`](arcproLab/scripts/play_skrl.py) — GUI eval (auto-detects latest checkpoint)
- [`arcproLab/scripts/play_straight.py`](arcproLab/scripts/play_straight.py) — physics test (500-step, prints speed table)
- [`debug.sh`](debug.sh) — calls `play_skrl.py`

---

## Next Steps (In Order)
1. **Restart training** — current session doesn't have physics fixes loaded. Kill tmux `training`, rerun `start_tmux_training.sh`
2. **Monitor `WPΔ_rew`** — should see consistent `> 0.5` values without the 2000+ spikes (fixes reduce bounce that was triggering race condition)
3. **Fix Bug 16 (PENDING)** — `spawn_wp_idx` reset race condition in `rewards.py`. Investigate `hasattr(env, 'reset_terminated')` guard — may not be firing correctly
4. **Once WPΔ_rew confirmed stable** — re-enable Phase 2 penalties (lateral_error, heading, smoothness, jerk) at low weights
5. **Phase 16 MARL** — once single-agent SKRL is confirmed working end-to-end

---

## ALWAYS READ BEFORE REWARD/PHYSICS CHANGES
`.planning/reward_tuning_history.md` — 16 issues documented. Do not repeat them.
