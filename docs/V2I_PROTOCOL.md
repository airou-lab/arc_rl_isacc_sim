# ARCPro V2I Protocol

**Version:** 0.2.0 (draft)
**Status:** Proposed — partially implemented (see §0)
**Supersedes:** `.planning/INTERSECTION_NODE_DESIGN.md` (cited by three files in
`arc_rl_isacc_policy`; never committed to either repo)

An open, tensor-native protocol for communication between autonomous vehicles
and infrastructure nodes at intersections, designed for reinforcement learning
in NVIDIA Isaac Sim and for deployment on F1Tenth-class hardware.

---

## 0. Relationship to `V2I_DESIGN.md`

`V2I_DESIGN.md` on branch `aaron/v2i_intersection` is **prior art, implemented
and CPU-tested (36 tests)**. This document extends it; it does not replace it.

What that branch already establishes, and this spec adopts unchanged:

- **Pluggable arbitration authority.** Signalized (`SignalizedScheduler`, an FSM)
  and reservation-based (`IntersectionNodeServer`, conflict-matrix) behind one
  `register_intent / query_go_signal / clear_agent / tick / active_intents`
  interface. Comparing the two paradigms under matched comms degradation is the
  research contribution.
- **`register_intent` is the actuation uplink.** A registered intent is standing
  demand for its approach group. A vehicle that registers is collaborating; one
  that never registers still receives correct fixed-time signals. Graceful
  degradation is a property of the design, not an afterthought.
- **`V2IMessageBuffer`** (`arcproLab/mdp/v2i_buffer.py`) already models latency,
  dropout, beacon rate, and staleness, returning `(held, valid, age)`. Ideal
  comms is the degenerate config. It is **written but not yet wired into the
  training loop**.
- **Extras contract.** Reward terms read only `env.extras`, never the
  `RoadManager` singleton.

What this document adds: the wire format, the normalization constants, the
staleness semantics that `V2I_DESIGN.md` §4.3 left as an open sync item, and the
channel-randomization scheme required for sim-to-real.

**Branch fork point is `c6c4d3c` (2026-06-20), before the SKRL migration.** The
branch must be rebased onto post-SKRL `main` before any of it ports. One design
element does not survive that rebase — see §8.

---

## 1. Why not an existing standard

SAE J2735 (BSM / SPaT / MAP) is the obvious candidate and is deliberately not
used here:

- **Paywalled.** Readers of the resulting work cannot inspect the message
  semantics without purchasing the standard.
- **ASN.1 / UPER encoded.** Requires a compiler toolchain and produces
  variable-length octet strings. Neither property survives contact with a
  batched GPU observation tensor.
- **Scaled for full-size vehicles.** Field ranges, timing granularity, and
  transmit cadence assume road speeds. This platform runs at 0.5–2.0 m/s with
  a 0.33 m wheelbase. §6 shows where that mismatch already caused a concrete
  error.

Section 10 maps each field defined here onto its nearest J2735 equivalent, so
the correspondence is documented without the dependency.

---

## 2. Design constraints

These come from the training stack and are non-negotiable for v0.

| # | Constraint | Source |
|---|---|---|
| C1 | Fixed message width; no dicts, no string IDs | Vectorized over `(num_envs, num_agents)` in IsaacLab |
| C2 | All fields pre-normalized to approximately `[-1, 1]` | `cfg_ppo["state_preprocessor"] = None` in `arcproLab/scripts/train_skrl.py` — nothing normalizes observations downstream |
| C3 | Spatial fields expressed in the receiver's frame | World-frame coordinates cause the policy to memorize a single intersection |
| C4 | Permutation-invariant across neighbouring vehicles | Agent ordering must not leak into the policy |
| C5 | Message width stable across all rollout stages | Widening the observation forces a full retrain; it is paid once |
| C6 | Sim-time only; no wall-clock | Headless training does not run at real time |
| C7 | The radio carries only what vision cannot obtain | R002 / D010 — see §2.1 |
| C8 | **No field may encode a fleet size or vehicle count capacity** | See §2.2 |

### 2.1 The provenance rule (C7)

Two entries in `.gsd/DECISIONS.md` appear to contradict each other until
their difference is isolated:

- **D010 — reverted.** Five forward waypoints injected into telemetry.
  Reverted because "providing numerical anticipation bypassed the requirement
  (R002) for the agent to learn HD Vision-based navigation."
- **D022 — accepted, marked non-revisable.** The same waypoint indices used
  to compute `waypoint_progress_reward`.

Identical data, opposite verdicts. The distinguishing factor is where it
enters: D010 placed it in the **actor's observation**, D022 in the **reward**.
The governing rule is therefore:

> Privileged, non-perceptual information may shape the reward or inform the
> critic. It must never enter the actor's observation.

This is the asymmetric actor-critic contract (§7), and it constrains this
protocol directly: any field the camera could have determined must come from
perception, not from the radio. A numerical distance delivered over the wire
would be D010 repeated.

**Division of information sources:**

| Information | Source | Rationale |
|---|---|---|
| Lane position, curvature | Frozen ResNet-18 | What the backbone is trained for |
| Distance to stop line | `VisualStopLineDetector` (cv2) | Visually determinable. **Absent from `main`** — see §5.2 |
| Stop-bar compliance (`go_signal`) | `GoSignalManager` (cv2 FSM) | Perception-derived; already in the observation at telemetry slot 1 |
| Signal phase, time-to-change | **Radio** | Not visually determinable with a frozen backbone |
| Conflicting vehicle behind occlusion | **Radio** | The substantive value of V2I |

Note that a cv2 estimate computed from the camera frame satisfies R002. D010's
defect was injecting *simulator ground-truth geometry*, not perception output.

**Backbone constraint.** D012 and D017 froze the ResNet-18 entirely, and the
MLP head cannot fine-tune it. A frozen ImageNet extractor is unlikely to encode
signal-phase state usefully in its 512 generic dimensions, so "the agent learns
to see the light" is not available on this architecture. This independently
supports phase arriving over radio.

### 2.2 Scalability (C8)

The physical experiment has **8 vehicles**, and the count is not a design
parameter. The system must scale to arbitrary N without a protocol revision.
Two consequences:

1. **All vehicles run one shared policy** — identical weights, each with its own
   observation. N is a batch dimension, not an architectural one. The later
   transition to MARL is a change of training scheme, not of message format.
2. **No normalization constant may be a capacity.** §3.1 states that changing a
   normalization constant invalidates every existing checkpoint. A constant like
   `N_MAX = 8` would therefore detonate the first time a ninth vehicle runs.
   Quantities that grow with fleet size use saturating normalization instead
   (§5, field 7).

---

## 3. Conventions

**Frames.** Isaac Sim world frame: `+X` right, `+Y` up, `Z` up. Heading in
radians, `0 = +X`, `π/2 = +Y`, counter-clockwise. An *approach heading* is the
direction a vehicle faces while driving **toward** the intersection centre.

**Control loop.** 50 Hz. Sim `dt = 0.002` with `decimation = 10`
(`arcproLab/arcpro_env_cfg.py`). One "step" below always means one 50 Hz
control step, i.e. 20 ms.

**Turn encoding.** On the wire, turns are one-hot. Internally the existing
`TurnCommand` integer encoding is retained: `LEFT = -1`, `STRAIGHT = 0`,
`RIGHT = 1`.

### 3.1 Normalization constants

These are **part of the protocol**, not implementation details. Changing any
of them shifts the policy's input distribution and invalidates every existing
checkpoint. They are versioned with the protocol.

| Constant | Value | Rationale |
|---|---|---|
| `V_MAX` | 2.0 m/s | Max observed chassis speed is 1.968 m/s (straight-line physics test) |
| `D_MAX` | 2.0 m | Spawn-to-junction-centre is 1.394 m on the current track (§3.2) |
| `T_MAX` | 10.0 s | Longest phase duration the node may advertise |
| `MAX_AGE_STEPS` | 10 | 200 ms at 50 Hz — 42% of the approach window (§6) |
| `PROTO_VERSION` | 2 | Integer, transmitted as `version / 16` |

`N_MAX` was removed in v0.2.0. It was a capacity constant and violated C8.

### 3.2 Junction geometry (resolved 2026-07-27)

The training junction is **`junction_18`**, named in the spawn comment in
`arcpro_env_cfg.py`. Its coordinates are recorded in
`openStreetUSD/no_graph_sim_clean_1x_flattened.usda` under
`sub_intersections/network/junction_18`, which is **stale 8× annotation
metadata inside a 1×-rescaled file** — the road geometry was resized in Phase
14-01 but this annotation subtree was not.

The divisor is confirmed by Arika's own viewer comment in `arcpro_env_cfg.py`:
`8x eye: (-120.0, 55.0, 10.0) -> 1x eye: (-15.0, 6.875, 1.25)` (55/8 = 6.875).

| Quantity | Raw (8×) | **1× world** |
|---|---|---|
| Junction centre (`AnalyticalPos`) | (-127.755, 55.000) | **(-15.969, 6.875)** |
| Crossing half-width | 5.3 | **0.66 m** |
| Lane offset from centreline | 1.75 | **0.219 m** |
| Approach stop-line gate | — | **≈ (-16.19, 6.22)** |

Verification: the 12 `laneGate_*` transforms are symmetric about
`AnalyticalPos` at ±5.3 on each axis with ±1.75 lane offset, confirming it is
the true centre in that coordinate space. At 1× this yields 0.66 m arms and a
0.219 m lane offset — consistent with an F1Tenth-scale track (0.28 m track
width).

**Derived distances** from the spawn at (-16.197, 5.50):

- spawn → stop line: **0.715 m**
- spawn → junction centre: **1.394 m**

**Consequences.**

1. `D_MAX = 2.0 m`, not 6.0. The earlier value was drawn from the discarded
   `V2VManager` centre of (-16.25, 0.35), which is ~5.8 m from the true centre
   and, since the robot spawns facing +Y (`rot=(0.7071,0,0,0.7071)`, yaw 90°),
   sits *behind* the vehicle. That constant is wrong and should not be reused.
2. The manual fallback at `track_manager.py:202`, `[-15.95, 6.19]`, is the
   **approach stop-line gate**, not the junction centre — its y matches the
   derived 6.22 to within 2.5 cm. It is correct for what it is and should not
   be treated as an intersection centre.
3. `SchedulerConfig.crossing_radius_m = 0.5` is close to the true 0.66 m and
   is a defensible default; consider raising it to 0.66.
4. The whole approach is **0.715 m ≈ 24 control steps ≈ 480 ms** at 1.5 m/s.
   Every timing constant in this protocol is derived from that window (§6).

---

## 4. Uplink — `IntentAnnouncement`

Vehicle → node. Broadcast. **The vehicle announces its intent; the node
arbitrates around it.** The node does not assign turns.

8 floats, expressed relative to the receiving node's reference frame.

| Idx | Field | Encoding | Range |
|---|---|---|---|
| 0 | `valid` | 1.0 if this slot holds a live announcement | {0, 1} |
| 1 | `d_to_node` | Euclidean distance to node centre / `D_MAX` | [0, 1] |
| 2 | `approach_sin` | `sin(θ)`, θ = vehicle heading relative to node reference axis | [-1, 1] |
| 3 | `approach_cos` | `cos(θ)` | [-1, 1] |
| 4 | `intent_left` | one-hot | {0, 1} |
| 5 | `intent_straight` | one-hot | {0, 1} |
| 6 | `intent_right` | one-hot | {0, 1} |
| 7 | `speed` | forward speed / `V_MAX` | [0, 1] |

**The uplink costs zero observation width.** Nothing here is a policy decision.
The turn intent already exists as the `RoadManager` mission token (telemetry
slot 0); position, heading, and speed are state the environment already holds.
The announcement is assembled env-side and consumed node-side. It never enters
the actor's observation, so adding it forces no retrain.

**The uplink is required, not optional.** Reservation-based arbitration *is* the
uplink — without `register_intent` there is nothing to grant, and the
signalized-vs-reservation comparison cannot be run at all. A downlink-only
system can express fixed-time signals and nothing else.

**On fields 2–3.** Heading is transmitted as `sin`/`cos` rather than a
discrete approach index. It wraps correctly, is continuous under small
perturbation, and degrades gracefully when a vehicle sits off-axis — the exact
case `IntersectionNodeServer._resolve_approach` currently handles by falling
back to a heuristic when the heading exceeds `APPROACH_TOLERANCE_RAD` (30°).
With a continuous encoding the node can resolve approach by nearest-axis
projection with no fallback path.

---

## 5. Downlink — `MovementAdvisory`

Node → vehicles. Broadcast, but **encoded per receiving vehicle in that
vehicle's own frame** (C3). One advisory serves every vehicle at the
intersection; each receives its own projection.

12 floats.

| Idx | Field | Encoding | Range | Live at |
|---|---|---|---|---|
| 0 | `valid` | 1.0 if received and `age ≤ MAX_AGE_STEPS` | {0, 1} | A |
| 1 | `age` | steps since receipt / `MAX_AGE_STEPS`, clipped | [0, 1] | A |
| 2 | `phase_permitted` | one-hot: own movement is cleared | {0, 1} | A |
| 3 | `phase_changing` | one-hot: about to change | {0, 1} | A |
| 4 | `phase_stopped` | one-hot: own movement is blocked | {0, 1} | A |
| 5 | `t_to_change` | seconds until phase changes / `T_MAX`, dead-reckoned (§6) | [0, 1] | A |
| 6–10 | `neighbour_summary[5]` | Pooled encoding of the neighbour set — see §5.1 | [0, 1] | B |
| 11 | `proto_ver` | `PROTO_VERSION / 16` | [0, 1] | A |

**On fields 2–4.** Phase is one-hot rather than a scalar. A single axis would
require the policy to learn that an intermediate value means "about to
change"; three indicator fields give three clean gradients.

**Distance to the stop line is deliberately absent.** It is visually
determinable, `VisualStopLineDetector` already computes it from the camera
frame, and delivering it numerically over the wire would repeat D010 (§2.1). It
enters the observation as a separate perception-derived term, not as a protocol
field.

### 5.1 The neighbour summary block (fields 6–10)

Five floats carrying everything the node knows about *other* vehicles. The
block is **opaque to the protocol** — its width is fixed, its interpretation
belongs to whatever encoder produces it.

This structure exists because of C4 and C8. The two obvious alternatives both
fail:

- **One block per neighbour** gives variable width when the neighbour count
  changes, and leaks ordering — car A in slot 0 today and slot 1 tomorrow
  produces different inputs for an identical world state.
- **Fixed-K nearest neighbours, sorted and zero-padded** hardcodes a capacity
  (violating C8) and is discontinuous when two vehicles swap rank.

Pooling solves both: encode each neighbour with a shared function φ, then
sum-pool. The result is permutation-invariant and independent of N by
construction.

```
neighbours → φ(each) → pool → neighbour_summary[5] → wire
             ^^^^^^^
      hand-designed now, learnable later
```

**Stage A** transmits zeros. **Stage B** uses a hand-designed φ occupying three
of the five dimensions:

| Sub-idx | Field | Encoding |
|---|---|---|
| 0 | `conflict_ttc` | time until nearest conflicting vehicle enters the crossing / `T_MAX`; 1.0 if none |
| 1 | `queue_ahead` | `q / (q + 4)` where `q` = vehicles ahead — saturating, see below |
| 2 | `conflict_load` | fraction of conflicting movements currently active |
| 3–4 | — | zero |

Aggregate encoding *is* pooling with a hand-designed φ. Replacing it with a
learned φ (DeepSets, or attention keyed on the ego vehicle) changes nothing the
vehicle policy can observe — same width, same invariances — so it costs no
retrain of the downstream stack. That is why the block is specified by width
rather than by field.

**On `queue_ahead`'s normalization.** `q / (q + 4)` is bounded in `[0, 1)`,
monotone, and never clips: 0 vehicles → 0.0, 4 → 0.5, 8 → 0.67, 20 → 0.83. It
encodes no assumption about fleet size, satisfying C8. A `q / N_MAX` form would
either clip or exceed `[0, 1]` and violate C2, which matters here because
nothing normalizes observations downstream.

**On `conflict_ttc`.** It describes a vehicle the camera may not be able to see
— approaching on a crossing arm, possibly occluded. This is the field that
justifies the radio existing at all, and it is unobtainable by any perception
path, so it does not engage C7.

### 5.2 The `go_signal` namespace collision (blocking for integration)

**`go_signal` currently names two different quantities, and they cannot share a
slot.** Anything integrating this protocol must resolve this first.

| | `main` (T3.2 / T3.3) | `aaron/v2i_intersection` |
|---|---|---|
| Meaning | *"Have I completed my mandatory stop?"* | *"Is my movement permitted?"* |
| Infrastructure | Stop **sign** / painted bar | Traffic **light** |
| Source | `GoSignalManager` — cv2 `VisualStopLineDetector` FSM (APPROACH → STOP → DEPART) | `IntersectionManager` phase FSM |
| Provenance | **Perception.** R002-compliant | **Radio.** Oracle in sim |
| Written to | `obs[:, 1]`, `env.extras["go_signal"]` | the same two |

Both write the same observation slot and the same `extras` key. Replacing one
with the other is not a merge — it silently changes what the policy is being
asked to learn, and it breaks the consumers hanging off the `main` semantics:
the throttle gate (`actions.py:216`), the reset hook (`events.py:111`), and the
`stationary` reward gate (`arcpro_env_cfg.py:191`).

It would also violate C7. `main`'s `go_signal` is perception output, which
§2.1 permits in the actor's observation. The FSM's is simulator ground truth. It
may reach the actor **only** through the channel model (§6), never as a direct
read, or it is D010 repeated.

**Resolution: they are separate fields.** Stop-bar compliance stays at slot 1
with its existing perception semantics. Signal phase arrives as advisory fields
2–4 (§5), degraded by the channel. A vehicle can simultaneously owe a stop-sign
dwell and hold a green light; one slot cannot express that.

**`main` cannot currently exercise either path.** `stop_line_detector.py` is
absent (see §11 item 6), so every consumer swallows the `ImportError` and
`go_signal` is pinned at 1.0. Restoring it is a prerequisite to integration, and
it fixes `main`'s intended behaviour rather than replacing it.

---

## 6. Channel model

The component that makes results transferable. Implemented as a ring buffer;
no Python loop, no wall-clock. **`arcproLab/mdp/v2i_buffer.py` on
`aaron/v2i_intersection` already implements this** (latency, dropout, beacon
interval, staleness; returns `(held, valid, age)`). What follows is what must
change in it, not a specification for new work.

### 6.1 The timescale must be derived from §3.2, not from automotive practice

`V2I_DESIGN.md` §2 and the `v2i_buffer.py` docstring both anchor on "10 Hz
SAE J2735-style beaconing → `beacon_interval = 5`". That anchor is drawn from
full-size vehicles, where an intersection approach lasts tens of seconds. At
F1Tenth 1× scale it does not survive:

| Parameter | Automotive-anchored | Travel at 1.5 m/s | vs. the 0.715 m approach |
|---|---|---|---|
| beacon period 100 ms | 5 steps | 0.15 m | ~5 advisories per approach |
| `max_age` 500 ms | 25 steps | 0.75 m | **105% — outlives the approach** |

An advisory held to a 500 ms `max_age` stays "valid" for longer than the
vehicle takes to clear the junction. `MAX_AGE_STEPS` is therefore **10**
(200 ms, 42% of the approach) — long enough to survive one missed beacon at
10 Hz with margin, short enough that expiry means something.

### 6.2 Channel parameters are randomized, not fixed

Sim-to-real transfer to physical hardware is in scope. Training against a single
latency value teaches the policy that value. Sampling per episode teaches
robustness, and the eventual measured radio profile then has to land *inside*
the range rather than match a number.

```
beacon_interval_steps ~ U{1, 5}        # 50 Hz down to 10 Hz
latency_steps         ~ U{0, 3}        # 0-60 ms, sampled per message
p_loss                ~ curriculum     # 0.0 -> 0.3, annealed
MAX_AGE_STEPS         = 10             # protocol constant, NOT randomized
```

`MAX_AGE_STEPS` stays fixed because it normalizes `age` in the observation
(§5, field 1); randomizing it would move the input distribution between
episodes and violate C2.

**Loss is a curriculum axis.** Train at `p_loss = 0` and anneal upward. The
reward stack has a documented history of destabilizing under abrupt penalty
changes (`.planning/reward_tuning_history.md`, 18 recorded issues);
introducing channel degradation gradually avoids repeating that.

### 6.3 Countdown fields are dead-reckoned inside the buffer

A held message is not merely stale, it is **wrong in a known direction**. If
`t_to_change` was 4.0 s at transmission and 10 steps have elapsed, the truth is
3.8 s but the held value still reads 4.0. The drift is linear in `age`.

The buffer corrects this before emitting, rather than leaving the policy to
learn arithmetic:

```
emitted[t_to_change] = max(0, held[t_to_change] - age_steps * dt)
```

This requires the buffer to know which fields are countdowns — a per-field
schema instead of an opaque `msg_dim`. That is the one structural change
`v2i_buffer.py` needs beyond wiring.

---

## 7. Asymmetric observation tap

The protocol is designed around the existing asymmetric actor-critic
(`arcproLab/agents/skrl_models.py`, after Pinto et al. 2017, arXiv:1710.06542).

```
encode_advisory(node_state, agent_state)  →  clean advisory vector
        │
        ├─► critic input   (clean + all vehicles' true intents + true conflict set)
        └─► channel(latency, loss) ─► actor input   (degraded, stale-flagged)
```

One encoder, tapped at two points. The critic's privileged channel is
literally the actor's channel before degradation, so there is no second code
path to keep in sync.

> **Note.** The critic is currently *less* informed than the actor:
> `arcproLab/mdp/observations.py:147-149` writes `lat_err` and `head_err` only
> when `masked=True` (the policy group), leaving both zero for the critic,
> while the actor additionally receives 512 vision dims. This inverts the
> premise. Adding privileged global intersection state to the critic is the
> correct fix and should land with this protocol, not separately.

---

## 8. Degraded mode

When `age > MAX_AGE_STEPS`, `valid` drops to 0 — the radio is effectively
down. The vehicle still has its camera, and `VisualStopLineDetector`
(`stop_line_detector.py`, 505 lines in `arc_rl_isacc_policy`, 439 on
`origin/Aaron_Summer_Testing_V1`, **absent from `main`**) continues to report
stop-line presence and range from the image.

**The cv2 detector is not a V2I fallback bolted on for this protocol — it is
already `main`'s primary stop-bar channel** (T3.2, §5.2), and it keeps that role
whether the radio is up or down. What degraded mode adds is that when the radio
drops, the detector's output becomes the *only* infrastructure signal the
vehicle has.

**The fallback is the classical detector, not the ResNet backbone.** The
backbone is frozen (D012, D017) and cannot learn to read infrastructure state;
the detector is deterministic and runs on the same D435i stream in deployment.

Degraded mode is therefore not a redundant copy of the same information — it
is a *reduction* in available information. With the radio up, the vehicle
knows phase, timing, and conflicting traffic. With it down, it knows only that
a stop line is ahead and how far. The policy must learn a conservative
fallback: approach, stop, and proceed on visual evidence alone.

### 8.1 The policy is feedforward; there is no recurrence

`V2I_DESIGN.md` Layer 4 specifies **RecurrentPPO**, with recurrence justified as
the mechanism that bridges dropout gaps. The SKRL migration (D017) replaced it
with feedforward PPO; `train_policy.py` now lives under `archive/sb3_legacy/`.
That design element does not survive the rebase, and the gap is covered
differently:

- **Held-value drift is closed-form**, not a memory problem. `true ≈ held −
  age·dt` is affine in two quantities the observation already carries, so a
  single linear layer computes it exactly. §6.3 moves the correction into the
  buffer, so the policy does not even have to learn it.
- **The strongest argument for recurrence is unavailable here anyway.** Its
  biggest payoff would be temporal vision — inferring a conflicting vehicle's
  approach speed across frames — and D012/D017 froze the backbone. An LSTM would
  sit downstream of single-frame features it cannot improve.
- **Cost is real.** SKRL RNN policies need sequence-aware rollout storage,
  hidden-state resets on episode boundaries, and BPTT windows. Against a reward
  stack with 18 documented tuning issues that is material instability risk. The
  hidden state is also a known sim-to-real hazard: it can encode simulator
  timing determinism that no physical radio reproduces.

**What is genuinely lost:** estimating channel *quality* over time ("this link
has been dropping; weight the camera more"). `age` gives a one-sample estimate.

**If the policy plateaus,** try stacking the last K advisory vectors before
reaching for an LSTM — finite history, feedforward, stable, and on a 12-float
message K=4 costs 48 floats.

---

## 9. Rollout stages

| Stage | Node behaviour | Uplink used | Agents | New capability |
|---|---|---|---|---|
| **A** | Fixed-time signal | No | 1+ | Vehicles learn to read phase + timing under latency and loss |
| **B** | Actuated — phase adapts to announced intent and queue | Yes | ≥2 | Uplink earns its existence; contention is real |

Stage A is meaningful with a single vehicle: a fixed-time signal blocks
regardless of who else is present, so the learning problem is real. This
contrasts with FCFS arbitration, where a lone agent is always queue head and
always receives GO — a single-agent test of that path exercises nothing.

Both stages run one shared policy across all vehicles (C8). The node is a
scripted arbiter throughout — signalized or reservation-based, swapped behind
the common interface (§0) to produce the paradigm comparison.

**Future direction, deliberately not designed for:** making the node itself an
RL agent that learns phase timing to optimize throughput. That is the MARL
transition, and nothing in this protocol is shaped by it. An earlier draft
allocated two message fields to it; they were removed in v0.2.0.

---

## 10. J2735 correspondence

Documented for reviewers; no dependency is implied.

| This protocol | J2735 nearest equivalent |
|---|---|
| `IntentAnnouncement` | `BasicSafetyMessage` + `RequestedItem` (SRM intent extension) |
| `MovementAdvisory.phase_*` | `SPAT.movementState.eventState` |
| `MovementAdvisory.t_to_change` | `SPAT.movementState.timing.minEndTime` |
| `neighbour_summary` | No direct equivalent; inferred from the received `BSM` set |
| `age` / `valid` | No equivalent — J2735 has no in-band staleness field |

The last row is the substantive divergence. Explicit staleness is what allows
the policy to learn a fallback behaviour rather than silently acting on
expired data.

---

## 11. Open items

1. **Sim-time source.** `SchedulerCore` currently uses `time.monotonic()` and
   a 15 s `intent_timeout`. Both must move to sim time
   (`episode_length_buf * dt`) during the port. The timeout is also
   automotive-scaled — 15 s is 31× the entire approach window (§3.2).
2. **Reward integration.** Introduce exactly one new term initially — a
   red-light violation penalty — with the throttle shield still active. The
   shield's activations should be counted and penalized so the policy learns
   not to need it, then annealed. Adding a shaping suite up front repeats the
   failure mode documented as Issues 10–13. `intersection_rewards.py` on the
   branch already implements gated terms; reconcile rather than rewrite.
3. **`T_MAX` provisional** pending the first training run. `D_MAX` is pinned
   (§3.2); `N_MAX` is gone (C8).
4. **Multi-agent observation mapping.** `V2I_DESIGN.md` §7 notes the obs mapping
   assumes N=1 while `RoadManager` and the FSM are `(B, N)`-ready. With 8
   vehicles this is a blocking limitation, not a deferral.
5. **Stale 8× annotation metadata.** The `sub_intersections/network` subtree in
   the USD was not rescaled in Phase 14-01. Anything reading it must divide by
   8. Worth either rescaling the subtree or deleting it, since it is a standing
   trap — it already produced one wrong constant.
6. **`stop_line_detector.py` must be restored to `main` before any of this is
   testable.** `go_signal_manager.py` imports it; the file exists only on
   `origin/Aaron_Summer_Testing_V1` (439 lines) and in `arc_rl_isacc_policy`
   (505 lines, newer). Until it lands, telemetry slot 1 is pinned at 1.0 by the
   bare `except` at `observations.py:74-77`, so **neither** the perception path
   (§5.2) **nor** the degraded-mode fallback (§8) can be exercised — the two
   paths this protocol depends on are both unreachable. This is the first fix,
   ahead of any V2I code: it restores the channel the rest of the design routes
   around.

---

## 12. Wire format (deployment)

For hardware, the float vectors above serialize to ROS2 message definitions —
plain text, human-readable, no ASN.1. `Ros2Transport` in the policy repo is the
intended carrier.

```
# MovementAdvisory.msg
uint8   proto_version
uint8   phase                  # 0=permitted 1=changing 2=stopped
float32 t_to_change            # seconds, unnormalized
float32 neighbour_summary[5]   # unnormalized; see §5.1
```

```
# IntentAnnouncement.msg
uint8   proto_version
uint8   intent                 # 0=left 1=straight 2=right
float32 d_to_node              # meters
float32 approach_heading       # radians, node frame
float32 speed                  # m/s
```

Normalization is applied at the observation boundary, not on the wire — the
wire carries SI units so the messages remain independently meaningful.
`valid` and `age` are added by the receiver's channel model (§6), not
transmitted.
