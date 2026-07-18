# ARCPro RL Training - Session Resume Context
**Date:** 2026-07-17 (Updated 18:47)

## Current Goal
**Achieve the Strict Target:** `ep_len_mean >= 600` AND `speed > 0.5 m/s` AND `WPΔ_rew > 0` (real waypoint progress via the reward signal, not the obs layer).

---

## Training Status
- **tmux session:** `training` — **ACTIVE** (started 18:45, running with all fixes below)
- **Log file:** `logs/skrl_phase1.log`
- **Step count at restart:** 0 (fresh restart with new reward config)

---

## Fixes Applied This Session (2026-07-17)

| Fix | File | What Changed |
|-----|------|--------------|
| **A** | `train_skrl.py` | Replaced `Trk: 0.0%` (permanently zero) with `WPs_cum` (raw WP count) |
| **B** | `train_skrl.py` + `rewards.py` | Log now shows `WPΔ_rew` — the actual per-step reward variable, not the unrelated obs cumulative variable |
| **C** | `rewards.py` | **Displacement gate**: reward only fires if robot moved ≥2cm — eliminates jitter farming via windowed-search index oscillation |
| **D** | `arcpro_env_cfg.py` | `progress_reward` weight `5.0 → 20.0` — navigation now earns ~34/step at 0.5 m/s |
| **Bug 5** | `arcpro_env_cfg.py` | `stationary` lambda uses `torch.abs()` — reverse driving no longer spuriously triggers penalty |
| **debug.sh** | `debug.sh` + `play_skrl.py` | Fixed broken call to nonexistent `verify_policy.py`; now calls `play_skrl.py` with auto-detection of latest checkpoint |

---

## New Log Format
```
Step N | Rew: X | Len: Y | Spd: Z | WPs_cum: A | WPΔ_rew: B
```
- `WPs_cum` — cumulative WPs from spawn this episode (obs variable, resets each episode)
- `WPΔ_rew` — per-step reward WP delta (rewards.py variable, displacement-gated)

**Critical threshold: `WPΔ_rew > 0.5` consistently = agent is genuinely navigating.**

---

## Key Files
- [`arcproLab/mdp/rewards.py`](arcproLab/mdp/rewards.py) — `waypoint_progress_reward()`: displacement gate + exposes `reward_wp_delta_step` to extras
- [`arcproLab/arcpro_env_cfg.py`](arcproLab/arcpro_env_cfg.py) — `progress_reward` weight=20.0, `stationary` uses abs()
- [`arcproLab/scripts/train_skrl.py`](arcproLab/scripts/train_skrl.py) — logs `WPs_cum` and `WPΔ_rew`
- [`arcproLab/scripts/play_skrl.py`](arcproLab/scripts/play_skrl.py) — GUI eval; `--checkpoint` defaults to auto-detect latest
- [`debug.sh`](debug.sh) — calls `play_skrl.py`; pass `--checkpoint /path` to override

---

## Next Steps (In Order)
1. ✅ Training restarted with all fixes
2. **Monitor `WPΔ_rew`** — if consistently > 0.5 by 100k steps, Fix C + D worked
3. **If `WPΔ_rew` stays 0** — check `reward_wp_delta_step` key is in extras (may need to inspect isaaclab wrapper's info dict passthrough)
4. **Once WPΔ_rew confirmed** — re-enable Phase 2 penalties (lateral_error, heading, smoothness, jerk) at low weights
5. **Known remaining issue (Bug 4)** — `spawn_wp_idx` reset race condition can cause occasional Rew=1000+ spikes; low priority until navigation is confirmed

---

## ALWAYS READ BEFORE REWARD CHANGES
`.planning/reward_tuning_history.md` — 14 issues documented; do not repeat them.
