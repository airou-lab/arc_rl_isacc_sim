---
name: rl-deep-audit
description: >-
  Deterministic 6-point forensic debugging runbook for RL, IsaacLab, and SKRL training plateaus,
  observation/memory aliasing, asymmetric actor-critic masking, MDP reward/termination ordering,
  and checkpoint rollback vs. fresh-start viability. Engineered specifically for Gemini Flash
  to catch silent multi-layer RL bugs that superficial audits miss.
---

# RL & IsaacLab Deep Debugging Runbook for Gemini Flash (`rl-deep-audit`)

Use this runbook whenever debugging RL training plateaus (e.g. stagnant `ep_len_mean`), unresponsive policy inputs (e.g. `turn_token` near-zero steering authority), unexpected terminations, or deciding whether to roll back to an earlier checkpoint vs. starting fresh (`--fresh`).

---

## 0. Four Anti-Patterns Gemini Flash MUST Avoid During Debugging

1. **Comment Blindness (Never Trust Comments Over Tensor Dataflow)**:
   - Do NOT assume a comment like `# Zero out privileged features for actor` or `# Preallocated buffer to prevent VRAM leak` works as intended.
   - Trace every tensor from **allocation** $\to$ **in-place mutation (`.copy_()`)** $\to$ **replay memory storage (`record_transition`)** $\to$ **network consumption (`compute()`)**.
2. **Single-Cause Premature Closure (Never Stop at the First Bug)**:
   - RL plateaus are almost always caused by **multiple compounding bugs** across wrapper, critic, reward, and geometry layers.
   - You MUST complete **all 6 Diagnostic Gates** below before concluding your diagnosis.
3. **Wrong Probe Index (Always Verify Exact Tensor Slice Offsets)**:
   - Never guess feature indices when probing a policy or critic head. Use the exact canonical index map in Section 1.
4. **Superficial Checkpoint Checks ("No NaNs = Healthy" Fallacy)**:
   - A checkpoint with zero `NaN`s and high cosine similarity is still **mathematically poisoned** if it was trained with aliased replay memory (`s_t == s_{t+1}`), crash-to-spawn value inversion, or zero-masked inputs on newly unmasked Critic weight columns.

---

## 1. Canonical Tensor & Telemetry Index Map (ARCPro / SKRL)

Always verify these exact indices when writing diagnostic probes:

- **Flattened Environment State (`25,100` dims)**:
  - `state[:, 0:25088]`: Downsampled RGB visual stream ($16 \times 56 \times 28$).
  - `state[:, 25088:25100]` (`state[:, -12:]`): 12-D onboard + privileged telemetry vector.
- **`ARCProActor.policy_head` Input (`76` dims)**:
  - `head_in[:, 0:64]`: 32 $(x, y)$ continuous keypoints from `SpatialSoftmax2d`.
  - `head_in[:, 64:76]`: 12-D telemetry vector (**telemetry slot $i$ is at policy-head index `64 + i`**).
- **`ARCProCritic.critic_head` Input (`12` dims)**:
  - `critic_in[:, 0:12]`: 12-D telemetry vector (**telemetry slot $i$ is at critic-head index `i`**).
- **12-D Telemetry Slot Definitions (`slot 0..11`)**:
  | Slot `i` | Actor Head Idx (`64+i`) | Feature Name | Consumer | Notes |
  | :---: | :---: | :--- | :--- | :--- |
  | `0` | **`64`** | `turn_token` | Actor + Critic | `-1.0` Left, `0.0` Straight, `+1.0` Right (NOT idx 66!) |
  | `1` | `65` | `dist_g` | Actor + Critic | Distance to active navigation gate |
  | `2` | `66` | `v_x` | Actor + Critic | Body longitudinal speed (m/s) |
  | `3` | `67` | `v_y` | Actor + Critic | Body lateral speed (m/s) |
  | `4` | `68` | `yaw_rate` | Actor + Critic | Body yaw rate $r$ (rad/s) |
  | `5` | `69` | `steer_angle` | Actor + Critic | Current steering joint angle |
  | `6, 7` | `70, 71` | `prev_action` | Actor + Critic | Previous `[steer, throttle]` action |
  | `8` | `72` | `lat_err` | **Critic ONLY** | Unmasked in `observations.py`; cloned & zeroed in `Actor.compute()` |
  | `9` | `73` | `head_err` | **Critic ONLY** | Unmasked in `observations.py`; cloned & zeroed in `Actor.compute()` |
  | `10` | `74` | `lookahead_kappa`| Actor + Critic | 5-point windowed max-magnitude lookahead curvature |
  | `11` | `75` | `distance` | **Critic ONLY** | Cumulative progress distance; cloned & zeroed in `Actor.compute()` |

---

## 2. The 6 Mandatory Forensic Debugging Gates

### Gate 1: Wrapper & Replay Buffer Pointer Non-Aliasing (`s_t` vs `s_{t+1}`)
- **Why bugs hide here**: SKRL's `SequentialTrainer.train()` holds a reference to `states` ($s_t$), calls `next_states = env.step(actions)`, and *only afterwards* calls `agent.record_transition(states=states, next_states=next_states)`. If the wrapper writes into a single static buffer (`self._flat_obs_buffer.copy_()`), `env.step()` overwrites `states` in-place with `next_states` (or with the post-reset spawn observation $s_{\text{spawn}}$ on crash steps!), causing $s_t \equiv s_{t+1}$ and poisoning $V(s_{\text{spawn}}) \approx -400$.
- **How Flash Must Debug It**:
  1. Inspect `SKRLFlattenWrapper._flatten_obs()` in `arcproLab/agents/skrl_wrappers.py`.
  2. Run or verify a 2-step pointer check:
     ```python
     obs_0, _ = env.reset()
     obs_1, _, _, _, _ = env.step(torch.zeros((env.num_envs, 2), device=env.device))
     assert obs_0.data_ptr() != obs_1.data_ptr(), "CRITICAL: s_t and s_{t+1} share memory!"
     ```
  3. Verify a **2-slot ping-pong ring buffer** (`self._flat_obs_buffers = [buf0, buf1]`, toggled via `self._buf_idx = (self._buf_idx + 1) % 2`) is used.

### Gate 2: Asymmetric Actor-Critic Masking & Gradient Flow
- **Why bugs hide here**: If privileged slots (`8, 9, 11`) are zeroed inside `mdp/observations.py`, `RandomMemory` receives `0.0` and the Critic is completely blind to lateral error, heading error, and progress distance. Conversely, if `Actor.compute()` zeroes `inputs["states"][:, 25088:25100]` in-place without `.clone()`, it mutates the shared minibatch tensor in `RandomMemory`.
- **How Flash Must Debug It**:
  1. Check `arcproLab/mdp/observations.py` (`_compute_telemetry`): confirm slots `8, 9, 10, 11` are **populated with live values** (never zeroed at the source).
  2. Check `arcproLab/agents/skrl_models.py` (`ARCProActor.compute`): confirm `vec = state[:, 25088:25100].clone()` is called before setting `vec[:, [8, 9, 11]] = 0.0`.
  3. Assert that backpropagating Critic output (`v.sum().backward()`) produces non-zero gradients on slots `[8, 9, 10, 11]`, while Actor output (`mean.sum().backward()`) has exact `0.0` gradient on slots `[8, 9, 11]`.

### Gate 3: IsaacLab Manager Execution Order (`terminations` -> `rewards` -> `observations`)
- **Why bugs hide here**: In IsaacLab's `ManagerBasedRLEnv.step()`, the execution sequence is strictly:
  `ActionManager` $\to$ `Physics` $\to$ `TerminationManager.compute()` $\to$ `RewardManager.compute()` $\to$ `reset_idx()` $\to$ `ObservationManager.compute()`.
  Any dictionary key (`env.extras["kappa"]`, `env.extras["dist_y"]`, etc.) written only inside `observations.py` is **1 step stale** (or `0.0` at Step 1) when read by `rewards.py`!
- **How Flash Must Debug It**:
  1. Grep for `extras` across `arcproLab/mdp/terminations.py`, `arcproLab/mdp/rewards.py`, and `arcproLab/mdp/observations.py`.
  2. Verify that every `env.extras["..."]` key read inside `rewards.py` (`dist_y`, `dist_w`, `dist_g`, `lat_err`, `head_err`, `kappa`) is computed and stored inside `terminations.py` (`white_line_contact`), which executes immediately *before* `RewardManager`.

### Gate 4: Zero-Error Steady-State Reward Equilibrium (Feedforward vs. Feedback)
- **Why bugs hide here**: Pure proportional-derivative steering guidance (`target_steer = -k_y * lat_err - k_psi * head_err`) evaluates to `0.0` when the vehicle enters a corner centered on the lane (`lat_err = 0, head_err = 0`). That means a high-weight expert reward (`weight = 100.0`) actively penalizes the policy for turning the wheel at corner entry until *after* the car has already washed out off the centerline!
- **How Flash Must Debug It**:
  1. Substitute `lat_err = 0.0, head_err = 0.0, kappa = +1.0` into every steering reward in `arcproLab/mdp/rewards.py`.
  2. Verify Ackermann kinematic curvature feedforward is added to `target_steer`:
     $$\delta_{\text{ff}} = \frac{\arctan(L \cdot \kappa_{\text{signed}})}{\delta_{\max}} \quad (L = 0.33\text{ m}, \; \delta_{\max} = 0.68\text{ rad})$$
     so `target_steer` commands `+0.39..+0.55` (left turn) at centered corner entry instead of `0.0`.
  3. Check spatial masks (`in_c1`, `in_inter`, `in_bend`, `in_c4`): ensure pre-braking speed targets (`curvature_speed_reward`) can activate `1.5m` early without activating curved steering targets while the car is still on the straight approach chute.

### Gate 5: Spatial Signal & Lookahead Window Alignment Against Track Geometry
- **Why bugs hide here**:
  1. **Premature `turn_token` in Straight Chutes**: If `RoadManager.get_turn_Direction()` sets `turn_token = -1.0` while the car is still `0.8m–1.3m` inside a straight double-yellow chute (`dist_y = 0.20m`), any left steering response immediately triggers `boundary_penalty` (`-160`) and yellow-line termination—teaching the policy to ignore `turn_token` (`weight ≈ 0`).
  2. **Single-Point Lookahead Overshoot**: A single-point curvature probe at `+2.0m` (`169` waypoints ahead) overshoots a `1.1m` (`95`-waypoint) $90^\circ$ corner arc, reading `lookahead_kappa = 0.0` during the `0.60m` turn-in window and mid-corner!
- **How Flash Must Debug It**:
  1. Compare `in_c1_turn`, `in_c2_turn`, `in_bend_turn`, and `in_c4_turn` bounding boxes in `arcproLab/mdp/road_manager.py` against physical curve entry coordinates (`Y = -6.10`, `X = -5.65`, `Y = +2.40`, `X = -14.75`).
  2. Verify `TrackManager.get_lookahead_curvature()` in `arcproLab/mdp/track_manager.py` samples a multi-point window (`[0.0, 0.25, 0.50, 0.75, 1.0] * step_offset`) and gathers the curvature with maximum absolute magnitude.

### Gate 6: Checkpoint Forensics & Rollback vs. Fresh Start (`--fresh`) Decision
- **Why bugs hide here**: Rolling back to an earlier checkpoint (`50k..350k`) wastes days of compute if that checkpoint lineage was trained under corrupted memory or masked Critic inputs.
- **How Flash Must Debug It**:
  1. Inspect candidate checkpoints (`torch.load(ckpt_path, map_location="cpu")`):
     - Check optimizer step count: `ckpt["optimizer"]["state"][0]["step"]`.
     - Check Critic input column L2 norms: `ckpt["critic"]["critic_head.0.weight"].norm(dim=0)` (slots `0..11`).
     - Check Actor telemetry column L2 norms: `ckpt["policy"]["policy_head.0.weight"][:, 64:76].norm(dim=0)` (especially index `64` = `turn_token` and index `74` = `lookahead_kappa`).
  2. **Mandatory `--fresh` Rule**: You MUST reject rollback and start fresh (`./start_tmux_training.sh --fresh`) if the checkpoint lineage was trained while:
     - `states` and `next_states` were aliased in replay memory (`Gate 1` failure), OR
     - Critic slots (`8, 9, 11`) were masked to `0.0` at the source (`Gate 2` failure), leaving untrained Kaiming noise on `critic_head.0.weight[:, [8, 9, 11]]`.
  3. When launching `--fresh`, always quarantine poisoned run folders into `archive/corrupted_<reason>_<date>/` and ensure `logs/sentinel_state.json` is cleared (`audited_checkpoints` keyed by `f"{run_dir.name}/{ckpt_path.name}"`).
