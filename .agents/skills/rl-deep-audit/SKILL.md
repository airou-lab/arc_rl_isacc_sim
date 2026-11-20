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

Always verify these exact indices against `arcproLab/mdp/track_topology.py` (`TELEMETRY_NAMES`, `ACTOR_MASKED_TELEMETRY_INDICES`) and `arcproLab/agents/skrl_models.py` (`VISUAL_FEATURE_DIM`):

- **Flattened Environment State (`25,100` dims)**:
  - `state[:, 0:25088]`: Flattened ResNet-18 spatial feature map (`512 x 7 x 7`, `VISUAL_FEATURE_DIM = 25088`).
  - `state[:, 25088:25100]` (`state[:, -12:]`): 12-D onboard + privileged telemetry vector (`TELEMETRY_DIM = 12`).
- **`ARCProActor.policy_head` Input (`76` dims)**:
  - `head_in[:, 0:64]`: 32 $(x, y)$ continuous keypoints from `SpatialSoftmax2d`.
  - `head_in[:, 64:76]`: 12-D telemetry vector (**telemetry slot $i$ is at policy-head index `64 + i`**).
- **`ARCProCritic.critic_head` Input (`12` dims)**:
  - `critic_in[:, 0:12]`: 12-D telemetry vector (**telemetry slot $i$ is at critic-head index `i`**; slot 11 `distance` is `asinh(d / 10.0)` compressed + running-stat normalized).
- **12-D Telemetry Slot Definitions (`slot 0..11`, enforced by `tests/test_agent_doc_contracts.py`)**:
  | Slot `i` | Actor Head Idx (`64+i`) | Feature Name | Consumer | Notes |
  | :---: | :---: | :--- | :--- | :--- |
  | `0` | `64` | `turn_token` | Actor + Critic | `-1.0` Left, `0.0` Straight, `+1.0` Right |
  | `1` | `65` | `go_signal` | Actor + Critic | `1.0` Go, `0.0` Stop (`GoSignalManager`) |
  | `2` | `66` | `goal_dist` | Actor + Critic | Distance to active navigation gate/goal |
  | `3` | `67` | `speed` | Actor + Critic | Body planar speed $\lVert v_{xy} \rVert$ (m/s) |
  | `4` | `68` | `yaw_rate` | Actor + Critic | Body yaw rate $r$ (rad/s) |
  | `5` | `69` | `prev_steer` | **Critic ONLY** | Masked in `ARCProActor._mask_actor_telemetry()` (`ACTOR_MASKED_TELEMETRY_INDICES`) |
  | `6` | `70` | `prev_drive` | **Critic ONLY** | Masked in `ARCProActor._mask_actor_telemetry()` (`ACTOR_MASKED_TELEMETRY_INDICES`) |
  | `7` | `71` | `lat_vel_b` | Actor + Critic | Body lateral velocity clamped to `[-2.0, 2.0]` (m/s) |
  | `8` | `72` | `lat_err` | **Critic ONLY** | Unmasked in `observations.py`; masked in `ARCProActor._mask_actor_telemetry()` |
  | `9` | `73` | `head_err` | **Critic ONLY** | Unmasked in `observations.py`; masked in `ARCProActor._mask_actor_telemetry()` |
  | `10` | `74` | `lookahead_kappa` | Actor + Critic | 5-point windowed max-magnitude lookahead curvature |
  | `11` | `75` | `distance` | **Critic ONLY** | Cumulative progress distance; masked for Actor, `asinh`-compressed for Critic |

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
- **Why bugs hide here**: If privileged slots (`5, 6, 8, 9, 11` in `ACTOR_MASKED_TELEMETRY_INDICES`) are zeroed inside `mdp/observations.py`, `RandomMemory` receives `0.0` and the Critic is blind to `prev_steer`, `prev_drive`, `lat_err`, `head_err`, and `distance`. Conversely, if `ARCProActor._mask_actor_telemetry()` zeroes `inputs["states"][:, 25088:25100]` in-place without `.clone()`, it mutates the shared minibatch tensor in `RandomMemory`.
- **How Flash Must Debug It**:
  1. Check `arcproLab/mdp/observations.py` (`_compute_telemetry`): confirm all 12 slots (`0..11`) are **populated with live values** (never zeroed at the source).
  2. Check `arcproLab/agents/skrl_models.py` (`ARCProActor._mask_actor_telemetry`): confirm `telemetry = raw_telemetry.clone()` is called before zeroing `ACTOR_MASKED_TELEMETRY_INDICES = (5, 6, 8, 9, 11)`.
  3. Assert that backpropagating Critic output (`v.sum().backward()`) produces non-zero gradients on slots `[5, 6, 8, 9, 10, 11]`, while Actor output (`mean.sum().backward()`) has exact `0.0` gradient on slots `[5, 6, 8, 9, 11]`.

### Gate 3: IsaacLab Manager Execution Order (`terminations` -> `rewards` -> `observations`)
- **Why bugs hide here**: In IsaacLab's `ManagerBasedRLEnv.step()`, the execution sequence is strictly:
  `ActionManager` $\to$ `Physics` $\to$ `TerminationManager.compute()` $\to$ `RewardManager.compute()` $\to$ `reset_idx()` $\to$ `ObservationManager.compute()`.
  Any dictionary key (`env.extras["kappa"]`, `env.extras["dist_y"]`, etc.) written only inside `observations.py` is **1 step stale** (or `0.0` at Step 1) when read by `rewards.py`!
- **How Flash Must Debug It**:
  1. Run `./venv/bin/pytest -m cpu tests/test_agent_doc_contracts.py` (AST-verifies `rewards.py` never consumes keys produced solely by `observations.py`).
  2. Verify every `env.extras["..."]` key read inside `rewards.py` (`dist_y`, `dist_w`, `dist_g`, `lat_err`, `head_err`, `kappa`, `turn_token`, `go_signal`) is computed inside `track_topology.compute_step_road_state()` / `terminations.white_line_contact()` before `RewardManager`.

### Gate 4: Zero-Error Steady-State Reward Equilibrium (Feedforward vs. Feedback)
- **Why bugs hide here**: Pure proportional-derivative steering guidance (`target_steer = -k_y * lat_err - k_psi * head_err`) evaluates to `0.0` when the vehicle enters a corner centered on the lane (`lat_err = 0, head_err = 0`). That means a high-weight expert reward (`weight = 100.0`) actively penalizes the policy for turning the wheel at corner entry until *after* the car has already washed out off the centerline!
- **How Flash Must Debug It**:
  1. Substitute `lat_err = 0.0, head_err = 0.0, kappa = +1.0` into every steering reward in `arcproLab/mdp/rewards.py`.
  2. Verify Ackermann kinematic curvature feedforward is added to `target_steer`:
     $$\delta_{\text{ff}} = \frac{\arctan(L \cdot \kappa)}{\delta_{\max}} \quad (L = 0.33\text{ m}, \; \delta_{\max} = 0.68\text{ rad})$$
     so `target_steer` commands `+0.39..+0.55` (left turn) at centered corner entry instead of `0.0` (do NOT flip sign via `track_dir = -1.0`).
  3. Check spatial masks (`in_c1_turn`, `in_c2_turn`, `in_bend_turn`, `in_c4_turn` in `arcproLab/mdp/track_topology.py`): ensure pre-braking speed targets (`curvature_speed_reward`) can activate early without activating curved steering targets while the car is still on the straight approach chute.

### Gate 5: Spatial Signal & Lookahead Window Alignment Against Track Geometry
- **Why bugs hide here**:
  1. **Premature `turn_token` in Straight Chutes**: If `RoadManager` sets `turn_token = -1.0` while the car is still `0.8m–1.3m` inside a straight double-yellow chute (`dist_y = 0.20m`), any left steering response immediately triggers `boundary_penalty` and yellow-line termination—teaching the policy to ignore `turn_token` (`weight ≈ 0`).
  2. **Single-Point Lookahead Overshoot**: A single-point curvature probe at `+2.0m` (`169` waypoints ahead) overshoots a `1.1m` (`95`-waypoint) $90^\circ$ corner arc, reading `lookahead_kappa = 0.0` during the `0.60m` turn-in window and mid-corner!
- **How Flash Must Debug It**:
  1. Inspect canonical corner predicates in `arcproLab/mdp/track_topology.py` (`in_c1_turn`, `in_c2_turn`, `in_bend_turn`, `in_c4_turn`, `in_multi_branch_intersection`) — never duplicate coordinate literals in downstream modules.
  2. Verify `TrackManager.get_lookahead_curvature()` in `arcproLab/mdp/track_manager.py` samples a multi-point window (`[0.0, 0.25, 0.50, 0.75, 1.0] * step_offset`) and gathers the curvature with maximum absolute magnitude.

### Gate 6: Checkpoint Forensics & Rollback vs. Fresh Start (`--fresh`) Decision
- **Why bugs hide here**: Rolling back to an earlier checkpoint wastes days of compute if that checkpoint lineage was trained under corrupted memory or masked Critic inputs.
- **How Flash Must Debug It**:
  1. Inspect candidate checkpoints (`torch.load(ckpt_path, map_location="cpu")` or `./venv/bin/python arcproLab/scripts/checkpoint_probe.py <ckpt> --equivariant --json`):
     - Check optimizer step count: `ckpt["optimizer"]["state"][0]["step"]` (or decoupled actor/critic optimizers).
     - Check Critic input column L2 norms: `ckpt["critic"]["critic_head.0.weight"].norm(dim=0)` (slots `0..11`).
     - Check Actor telemetry column L2 norms: `ckpt["policy"]["policy_head.0.weight"][:, 64:76].norm(dim=0)` (especially index `64` = `turn_token`, index `65` = `go_signal`, and index `74` = `lookahead_kappa`; masked slots `[69, 70, 72, 73, 75]` should remain inactive).
  2. **Mandatory `--fresh` Rule**: You MUST reject rollback and start fresh (`./start_tmux_training.sh --fresh`) if the checkpoint lineage was trained while:
     - `states` and `next_states` were aliased in replay memory (`Gate 1` failure), OR
     - Critic slots (`5, 6, 8, 9, 11`) were masked to `0.0` at the source (`Gate 2` failure), leaving untrained Kaiming noise on `critic_head.0.weight`.
  3. When launching `--fresh`, always quarantine poisoned run folders into `archive/corrupted_<reason>_<date>/` and ensure `logs/sentinel_state.json` is cleared (`audited_checkpoints` keyed by `f"{run_dir.name}/{ckpt_path.name}"`).
