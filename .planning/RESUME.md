# Session Resume Guide

## 1. Current Status
- Training **running** in `tmux: training` at **Step ~7,300** as of **2026-07-31 ~17:18 CST**.
- **128 parallel environments** on RTX 3060 (~8.2 GB / 12 GB VRAM, 87% GPU util).
- Fresh restart after **11-issue audit fix + refactor** session (see Section 2 below).
- Rewards stabilised at **~-45 to -60** (vs -1400 before fixes) — early but healthy.
- Speed **~0.18 m/s** (vs 0.08 m/s before tanh removal) — trending right.

## 2. All Changes Applied This Session (2026-07-31)

### A — 11 Bugs Fixed (Restart Required, Already Done)

| # | File | Change |
|---|---|---|
| 1 | `arcpro_env_cfg.py` | Removed `tanh` from progress_reward → linear `weight=30.0` |
| 2 | `mdp/events.py` | Re-enabled spawn DR: ±0.3m X, ±5° yaw |
| 3 | `mdp/terminations.py` | Stagnation: `<0.1@1000` → `<0.2@200` steps |
| 4 | `mdp/rewards.py` | Boundary penalty zone: `0.40m → 0.20m` |
| 9 | `mdp/observations.py` | go_signal cv2 loop bypassed (was 6-32× slowdown) |
| 10 | `scripts/train_skrl.py` | `kl_threshold: 0.008 → 0.02` |
| 11 | `scripts/train_skrl.py` | `entropy_loss_scale: 0.001 → 0.005` |
| 12 | `mdp/observations.py` | Slot 7 documented as dead slot |
| 13 | `mdp/observations.py` | Speed ÷2.0, yaw_rate ÷5.0 normalised |
| 16 | `mdp/observations.py` | Distance clamped ÷50.0 [0,1] |

### B — Refactor (Zero Behavior Change, Already Done)

| File | What Changed |
|---|---|
| `mdp/observations.py` | Monolith `_compute_telemetry` split into 6 named helpers: `_obs_turn_token`, `_obs_go_signal`, `_obs_ego_state`, `_obs_last_actions`, `_obs_track_geometry`, `_obs_track_progress`. Module-level normalization constants added. |
| `mdp/rewards.py` | Module-level tuning constants added (`BOUNDARY_WARN_THRESHOLD_M`, `TERMINATION_PENALTY_VALUE`, etc.). Three anonymous lambdas extracted to named functions: `survival_bonus`, `action_steer_penalty`, `action_drive_reward`. |
| `arcpro_env_cfg.py` | RewardCfg now uses named `mdp_rew.*` references instead of inline lambdas. |

## 3. Current Reward Config (Phase 1 Curriculum)
| Term | Weight | Effective/step | Notes |
|---|---|---|---|
| `survival_bonus` | 1.0 | +1 | `mdp_rew.survival_bonus` |
| `termination_penalty` | 20.0 | -1000 on crash | raw=-50, weight=20 |
| `progress_reward` | 30.0 | linear ~+36 at 0.5 m/s | NO tanh — direct speed gradient |
| `action_steer_penalty` | -0.5 | 0 to -0.5 | `mdp_rew.action_steer_penalty` |
| `action_drive_reward` | 0.5 | -0.5 to +0.5 | `mdp_rew.action_drive_reward` |
| `stationary` | 15.0 | -15 if spd<0.2 after step 50 | constant `STATIONARY_SPEED_THRESH_MPS=0.2` |
| `boundary` | 0.4 | -40 if dist<0.20m to wall | constant `BOUNDARY_WARN_THRESHOLD_M=0.20` |
| `lateral_error` | **0.0** | disabled | Phase 1 |
| `heading` | **0.0** | disabled | Phase 1 |
| `smoothness` | **0.0** | disabled | Phase 1 |
| `jerk` | **0.0** | disabled | Phase 1 |

## 4. Key Constants — One Place to Tune

**`mdp/rewards.py` top-of-file constants:**
```python
BOUNDARY_WARN_THRESHOLD_M   = 0.20   # wall warning zone
BOUNDARY_PENALTY_VALUE      = -100.0 # raw; × weight=0.4 = -40/step
TERMINATION_PENALTY_VALUE   = -50.0  # raw; × weight=20 = -1000/crash
STATIONARY_SPEED_THRESH_MPS = 0.2    # speed floor
STATIONARY_GRACE_STEPS      = 50     # grace before penalty fires
```

**`mdp/observations.py` top-of-file constants:**
```python
OBS_MAX_SPEED_MPS    = 2.0   # slot 3 normalization
OBS_MAX_YAW_RATE_RPS = 5.0   # slot 4 normalization
OBS_MAX_DISTANCE_M   = 50.0  # slot 11 normalization
```

## 5. Active Issues / Deferred
- **USD Steering Joints:** Rear wheels steer (inverted mesh axis). Camera/termination use -X convention to compensate. Deferred — not a training blocker.
- **go_signal bypassed:** Line `# go_signal = mgr.update(images)` in `_obs_go_signal()`. Re-enable for Phase 3+ intersection curriculum.
- **Obs Slot 7 dead:** `obs[:,7]` always 0 — documented, network ignores it.
- **WPs_cum telemetry undercount:** Backward wander wraps negative delta → ~3× undercount in logs. Telemetry only, rewards unaffected.

## 6. Next Steps for New Agent
1. **Check step throughput:** `tmux capture-pane -pt training:0.0 | tail -n 5`. Should see >10 steps/sec.
2. **Watch speed trend:** Target `Spd > 0.3` within first 50k steps.
3. **500k Milestone:** Evaluate `ep_rew_mean`, `speed_mps`, `ep_len_mean`. Target: `ep_len_mean >= 200` AND `speed_mps > 0.3`.
4. **Phase 2 Curriculum:** Once `ep_len_mean > 400` consistently → re-enable `heading` (weight 1.0) and `smoothness` (weight 1.0) in `arcpro_env_cfg.py`.
5. **Before ANY reward change:** Read `.planning/reward_tuning_history.md` first.

## 7. File Map (What Does What)
| File | Role |
|---|---|
| `arcproLab/arcpro_env_cfg.py` | Env config: scene, rewards, terminations, PPO hooks |
| `arcproLab/mdp/rewards.py` | All reward functions + tuning constants |
| `arcproLab/mdp/observations.py` | 6-function obs pipeline + normalization constants |
| `arcproLab/mdp/terminations.py` | All termination conditions |
| `arcproLab/mdp/events.py` | Spawn/reset logic + domain randomization |
| `arcproLab/mdp/track_manager.py` | Waypoint tracking, boundary detection |
| `arcproLab/mdp/go_signal_manager.py` | Stop-line FSM (bypassed in Phase 1) |
| `arcproLab/scripts/train_skrl.py` | SKRL PPO training entry point + hyperparams |
| `arcproLab/agents/skrl_models.py` | Actor (GaussianMixin) + Critic (DeterministicMixin) |
| `arcproLab/agents/skrl_wrappers.py` | SKRLFlattenWrapper: runs frozen ResNet per step |
| `start_tmux_training.sh` | Launches tmux training + watchdog |

*ALWAYS read this file AND `.planning/reward_tuning_history.md` before modifying rewards or physics!*
