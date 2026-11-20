# Reward & Architecture Failure-Mode Ledger

> **Verbatim Historical Archive:** The complete chronological log of Issues 1–120 (177 KB, Jul–Nov 2026) is preserved verbatim in [`archive/reward_tuning_history_full_issues_1_120.md`](../archive/reward_tuning_history_full_issues_1_120.md).
> **Rule:** Code and `pytest -m cpu` contract tests override Markdown on conflict. Consult this ledger before changing rewards, terminations, observation masking, or curriculum spawns to avoid repeating known failure loops.

---

## 1. Distilled Anti-Regression Invariants (Issues 1–116)

| Failure Class | Issue Nos. | Root Cause That Repeatedly Broke Training | Mandatory Invariant / Fix |
| :--- | :--- | :--- | :--- |
| **Suicide / Endless-Runner Loophole** | 1–6, 11, 19, 40, 61, 69, 79, 82 | Unbounded linear speed/progress rewards or oversized step penalties (`boundary_penalty` covering center of 0.50m lane, or `stationary` > `termination`) made crashing faster mathematically optimal. | Keep `termination_penalty` (`-400.0`) substantial enough to wipe out short-sprint profits, keep a zero-penalty center corridor (`BOUNDARY_WARN_MARGIN = 0.170m`), and never let step penalties make living worse than crashing. |
| **Stationary / Donut / Spinning-Top Farming** | 8, 14, 27, 34, 35, 37, 38, 50, 52, 64, 66–68, 83–86 | Rewarding raw body velocity, survival existence, `abs(throttle)`, or positive `steer^2` allowed parking or spinning in circles with `WPs_cum = 0`. | Progress must use centerline waypoint advancement (`waypoint_progress_reward`), gate shaping rewards by velocity (`clamp(speed / 0.20, 0, 1)`), and pair with `stagnation_termination` (`25 WPs / 100 steps` while `go_signal > 0.5`). |
| **IsaacLab `dt` Scaling Illusion** | 7, 73, 75 | `RewTerm` multiplies outputs by `step_dt = 0.02`; blindly multiplying all weights by `50x` caused over-penalization collapse, while raw return scales (`-100`, `-50`) multiplied by large weights crushed positive rewards. | Always verify effective per-step AND per-second reward budgets (`weight * raw_return * step_dt`) before changing any single weight. |
| **F1Tenth Root `+X` Backwards / Yaw `+pi`** | 31, 54, 56–58, 70 | The F1Tenth USD physics root `+X` points toward the rear wing (`-X` is nose-forward). Missing `+pi` yaw offset or wrong velocity sign inverted heading reward and triggered instant gate kills. | Keep forward drive velocity sign and `+pi` yaw / `track_dir` alignment consistent across `observations.py`, `rewards.py`, and `terminations.py` via `track_topology.compute_step_road_state()`. |
| **Kinematics & Elevation Limits** | 15, 17, 32, 43, 47, 48, 104, 117 | Frictionless tires, 5 Nm wheelie torque, RWD front-wheel lockup, ±35° PhysX steer joint clamping, `max_height = 0.50m` on a `Z = 2.50m` 3D hill, or `ground_contact = 1.0N` killed cars during suspension sag. | Maintain AWD, tire friction (`1.2 / 0.8`), ±55° steer joints (`max_steering_angle = 0.68 rad`), `max_height = 3.50m`, 20-step spawn grace period, and `ground_contact` tolerance (`50.0N`). |
| **Spatial Pooling & Telemetry Cheating** | 25, 36, 91, 93, 96 | ResNet `AdaptiveAvgPool2d((1,1))` destroyed left/right geometry (0.9974 cosine similarity); leaking `lat_err`/`head_err`/`distance` to the Actor allowed open-loop memorization; `play_skrl.py` `* 0.30` cap hid steering capability. | Use `SpatialSoftmax2d` keypoints (`skrl_models.py`), populate all 12 telemetry slots in `observations.py` for the privileged Critic, and mask `ACTOR_MASKED_TELEMETRY_INDICES = (5, 6, 8, 9, 11)` inside `ARCProActor._mask_actor_telemetry()` on a `.clone()`. |
| **Curriculum Blind Spots & Apex Immersion** | 95, 99, 113 | 100% start-line spawning starved corner transitions; spawning mid-apex from standstill with 0 steer had 0% cold-start survival; fleet rolling averages hid per-sector failures. | Never spawn inside a curve apex; give every sector ≥1.5m straight runway (`SECTOR_DEFS` in `track_topology.py`), keep all 5 sectors active, and evaluate with deterministic per-sector audits (`checkpoint_auditor.py`). |
| **Permissive Boxes & Double-Yellow Masking** | 107, 112, 116 | Wide rectangular permissive boxes zeroed yellow-line penalties/resets, teaching the car to drive into oncoming lanes. | Double-yellow median is **never** permeable anywhere; restrict white-line permeability strictly to `in_branch_permeable_gate()` (stop bar + active multi-branch straight/right exits). |
| **VRAM Leak & Tmux Suicide** | 110 | `ObservationManager` cloning RGB images (`+5.56 MB/step`) caused 12 GB OOM; synchronous `tmux kill-session` inside sentinel killed the sentinel itself. | Read camera RGB directly from static sensor GPU buffer in `SKRLFlattenWrapper` using preallocated 2-slot ping-pong buffers; always restart via detached `start_new_session=True` + stripped `TMUX`. |

---

## 2. Active Modern-Era Ledger (21k Spatial Softmax / Equivariant / M006 Era)

### Issue 117: Spatial Softmax Keypoints, Fused YUV & 3D Elevation Unlock
- **Root Cause:** Dense spatial FC layers (`453k` params) were sample-inefficient and struggled to separate yellow median vs. white lines under glare; `max_height = 0.50m` falsely killed climbs on the `Z = 2.50m` USD hill.
- **Fix:** Deployed Levine et al. `SpatialSoftmax2d` (`32` continuous `(x, y)` keypoints, `21,476` params), BT.601 YUV option, `max_height = 3.50m`, and preserved `15°` forward-bumper camera mount `pos=(0.22, 0.0, 0.18)`.
- **Invariant:** Pre-Issue-117 `453k` checkpoints in `archive/landmark_checkpoints/` are state-dict incompatible with the `21k` model.

### Issue 118: Sentinel Destructive Pruning & Incompatible Landmark Rollback Loop
- **Root Cause:** Sentinel 2 executed `shutil.rmtree` on active runs and fell back to incompatible `453k` landmarks, triggering an infinite crash/restart loop.
- **Fix:** Disabled destructive checkpoint pruning permanently, cleared obsolete `453k` fallbacks, and lowered Sentinel 2 apex-cliff threshold to `300` steps.

### Issue 119: Ping-Pong Observation Non-Aliasing, True AAC Unmasking & Zero-Delay `kappa`
- **Root Cause (5 compounding bugs):**
  1. Single static `_flat_obs_buffer` overwritten in-place made `s_t == s_{t+1}` inside SKRL `RandomMemory` (assigning `-400` crash penalties to post-reset spawn states).
  2. `observations.py` zeroed slots `[8, 9, 11]` at the source, blinding `ARCProCritic`.
  3. `intersection_expert_reward` lacked Ackermann curvature feedforward (`delta_ff = atan(L * kappa) / delta_max`) and read 1-step stale `env.extras["kappa"]` from `observations.py`.
  4. Premature `turn_token = -1.0` inside straight double-yellow chutes trained `ARCProActor` to ignore `turn_token`.
  5. Single-point `+2.0m` lookahead overshot 1.1m 90° corners.
- **Fix:** 2-slot ping-pong ring buffer (`skrl_wrappers.py`), live 12-D `observations.py` + clone-and-mask inside `ARCProActor`, `compute_step_road_state()` called inside `terminations.py` before `RewardManager`, Ackermann feedforward, track-aligned turn boxes, and 5-point windowed max-magnitude `get_lookahead_curvature()`. Mandatory `--fresh` rule when buffer aliasing or blind Critic poisoned a lineage.

### Issue 120: Autoregressive `prev_drive` Lockup, Sector 4 Runway Spawn & Stagnation Reset Fix
- **Root Cause:** Unmasked `prev_drive` (slot 6) created an autoregressive braking lockup (`0.06–0.12 m/s`); gating `lookahead_kappa` on `|turn_token| > 0.1` blinded straightaway pre-braking; Sector 4 spawned at `(-6.000, -7.118)` already inside curve/turn-token; `intersection_expert_reward` lacked a velocity gate; `stagnation_termination` copied previous episode's `cum_wp` on `episode_length_buf == 1`.
- **Fix:** Canonical `ACTOR_MASKED_TELEMETRY_INDICES = (5, 6, 8, 9, 11)` in `track_topology.py`, continuous unmasked `lookahead_kappa` (slot 10), Sector 4 moved to `(-7.000, -7.100)` straight runway, velocity-gated `intersection_expert_reward`, `stagnation_termination` raised to `25 WPs / 100 steps` with zero-reset on `episode_length_buf <= 1`, and `SKRLFlattenWrapper.reset()` fix.

### Issue 121: Intersection Wrong-Lane Entry & Guidance Curvature Sign Inversion
- **Root Cause:** Multiplying `kappa` by `track_dir = -1.0` inverted Ackermann feedforward steering (`delta_ff < 0`, steering RIGHT into a LEFT turn); oversized `in_inter_box` (`Y <= -2.20`) and symmetric `|dist_y - dist_w|` centering rewarded driving centered inside the oncoming West lane (`>+400 pts/step`).
- **Fix:** Used `kappa` directly without `-1.0` flip in `intersection_expert_reward`, sharpened `steer_match` to `-12.0`, added explicit `in_wrong_lane_intersection()` fatal termination + negative centering penalty, and unified spatial predicates inside `track_topology.py`.

### Issue 122: Corner 1 Exit Turn-Token Cliff, 48-Env Lighting/Overlap & Eval Symmetry Bugs
- **Root Cause:**
  1. Abrupt `in_c1_turn` cutoff dropped `turn_token` mid-exit while centrifugal momentum remained.
  2. `env_spacing = 30.0m` overlapped the `44.74m x 42.90m` track across 48 envs (casting hover shadows) and cloned 48 `DistantLight`s inside the per-env Track prim (`Bot_L = 235–245` vs 1-env).
  3. `play_skrl.py` / `checkpoint_auditor.py` left `training=True` DR active during deterministic eval; `sentinel_utils.probe_checkpoint_bias` omitted `--equivariant`; `ARCProCritic` running stats used `persistent=False`.
- **Fix:** Unified single-source `track_topology.py` + repaired `track_boundaries_1x.npz` (`0` violations at `0.170m`), decoupled Actor/Critic optimizers + `RunningStandardScaler` + `asinh` Critic distance compression + smooth `tanh` `log_std`, equivariant frontend `G(I)` + parity head + `L_sym`, persistent Critic stats, `--equivariant` CPU probe, and `M006` Multi-Branch Gates 1–3 (`tests/test_multi_branch_gates.py`, `44/44` passing).
