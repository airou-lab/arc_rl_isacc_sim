# ARCPro V2I Protocol

**Version:** 0.1.0 (draft)
**Status:** Proposed — not yet implemented
**Supersedes:** `.planning/INTERSECTION_NODE_DESIGN.md` (referenced by `arc_rl_isacc_policy`, never committed to either repo)

An open, tensor-native protocol for communication between autonomous vehicles
and infrastructure nodes at intersections, designed for reinforcement learning
in NVIDIA Isaac Sim and for deployment on F1Tenth-class hardware.

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
  a 0.33 m wheelbase.

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
| Distance to stop line | `VisualStopLineDetector` (cv2) | Visually determinable; already implemented |
| Signal phase, time-to-change | **Radio** | Not visually determinable |
| Conflicting vehicle behind occlusion | **Radio** | The substantive value of V2I |

Note that a cv2 estimate computed from the camera frame satisfies R002. D010's
defect was injecting *simulator ground-truth geometry*, not perception output.

**Backbone constraint.** D012 and D017 froze the ResNet-18 entirely, and the
MLP head cannot fine-tune it. A frozen ImageNet extractor is unlikely to encode
signal-phase state usefully in its 512 generic dimensions, so "the agent learns
to see the light" is not available on this architecture. This independently
supports phase arriving over radio.

**On C5:** Stage A uses 7 of the 12 downlink fields. The remaining 5 are
allocated now and transmitted as zero. This is intentional. Shipping a narrow
message and widening it at Stage B would force a second retrain from scratch.

---

## 3. Conventions

**Frames.** Isaac Sim world frame: `+X` right, `+Y` up, `Z` up. Heading in
radians, `0 = +X`, `π/2 = +Y`, counter-clockwise. An *approach heading* is the
direction a vehicle faces while driving **toward** the intersection centre.

**Control loop.** 50 Hz. Sim `dt = 0.002` with `decimation = 10`
(`arcproLab/arcpro_env_cfg.py`). One "step" below always means one 50 Hz
control step.

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
| `N_MAX` | 4 | Maximum vehicles queued per intersection |
| `MAX_AGE_STEPS` | 25 | 500 ms at 50 Hz; beyond this an advisory is stale |
| `PROTO_VERSION` | 1 | Integer, transmitted as `version / 16` |

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
| 5 | `t_to_change` | seconds until phase changes / `T_MAX`, clipped | [0, 1] | A |
| 6 | `conflict_ttc` | time until nearest conflicting vehicle enters the crossing / `T_MAX`; 1.0 if none | [0, 1] | B |
| 7 | `queue_ahead` | vehicles ahead of me / `N_MAX` | [0, 1] | B |
| 8 | `conflict_load` | fraction of conflicting movements currently active | [0, 1] | B |
| 9 | `slot_open` | seconds until my reservation window opens / `T_MAX` | [0, 1] | C |
| 10 | `slot_dur` | reservation window duration / `T_MAX` | [0, 1] | C |
| 11 | `proto_ver` | `PROTO_VERSION / 16` | [0, 1] | A |

**On fields 2–4.** Phase is one-hot rather than a scalar. A single axis would
require the policy to learn that an intermediate value means "about to
change"; three indicator fields give three clean gradients.

**On field 6.** `conflict_ttc` describes a vehicle the camera may not be able
to see — approaching on a crossing arm, possibly occluded. This is the field
that justifies the radio existing at all, and it is unobtainable by any
perception path, so it does not engage C7.

**Distance to the stop line is deliberately absent from this message.** It is
visually determinable, `VisualStopLineDetector` already computes it from the
camera frame, and delivering it numerically over the wire would repeat D010
(§2.1). It enters the observation as a separate perception-derived term, not
as a protocol field.

**On fields 7–8.** Aggregate rather than per-neighbour, which makes the
message permutation-invariant and fixed-width by construction (C1, C4). The
cost is loss of per-neighbour detail: sufficient for go/no-go arbitration,
insufficient for gap acceptance or tight merging. See §11.

---

## 6. Channel model

The component that makes results transferable. Implemented as a ring buffer;
no Python loop, no wall-clock.

> **⚠ The constants below are superseded and must not be implemented as
> written.** They were drafted before the junction geometry was resolved in
> §3.2 and do not survive it. At the resolved scale the spawn→stop-line
> approach is 0.715 m ≈ **480 ms** at 1.5 m/s, so:
>
> | Parameter | Drafted | Travel at 1.5 m/s | vs. 0.715 m approach |
> |---|---|---|---|
> | worst-case staleness (period + latency) | 200 ms | 0.30 m | 42% |
> | `max_age_steps = 25` | 500 ms | 0.75 m | **105% — exceeds it** |
>
> An advisory held to `max_age` outlives the entire approach: the vehicle
> would clear the junction still treating it as valid. The vehicle also
> receives only ~4–5 advisories per approach at a 100 ms period.
>
> These values were carried over from full-scale automotive intuition
> (approaches measured in tens of seconds). F1Tenth 1× geometry is ~20×
> tighter and the timescale does not close. Re-derive from §3.2 before
> implementing — expect roughly 50 Hz broadcast, 1–2 step latency,
> `max_age_steps` ≈ 5. Exact values depend on whether sim-to-real transfer
> is in scope; if it is, they should match a measured radio profile rather
> than being chosen for convenience.

```
# SUPERSEDED — see the note above. Retained to show what was assumed.
broadcast_period_steps = 5           # 10 Hz on the 50 Hz control loop
latency_steps          = U{1..5}     # 20–100 ms, sampled per message
p_loss                 = 0.05        # per message, independent
max_age_steps          = 25          # 500 ms → valid flips to 0
```

Buffer shaped `(num_envs, num_agents, horizon, D_msg)` with a parallel
`delivery_step` tensor. Each control step, deliver every message whose
`delivery_step == t` and which survived its loss draw. A vehicle holds its
last-received advisory; `age` increments each step until something replaces
it. Latency and loss reduce to index arithmetic and boolean masking.

**Loss is a curriculum axis, not a constant.** Train at `p_loss = 0` and
anneal upward. The reward stack has a documented history of destabilizing
under abrupt penalty changes (see `.planning/reward_tuning_history.md`,
18 recorded issues); introducing channel degradation gradually avoids
repeating that.

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
(`stop_line_detector.py`, 505 lines, currently in `arc_rl_isacc_policy`)
continues to report stop-line presence and range from the image.

**The fallback path is the classical cv2 detector, not the ResNet backbone.**
The backbone is frozen (D012, D017) and cannot learn to read infrastructure
state; the detector is deterministic and runs on the same D435i stream in
deployment.

Degraded mode is therefore not a redundant copy of the same information — it
is a *reduction* in available information. With the radio up, the vehicle
knows phase, timing, and conflicting traffic. With it down, it knows only that
a stop line is ahead and how far. The policy must learn a conservative
fallback: approach, stop, and proceed on visual evidence alone. That asymmetry
between modes is what makes the behaviour worth learning, and it gives the
visual detector a defined role rather than leaving it as orphaned code.

---

## 9. Rollout stages

| Stage | Node behaviour | Uplink used | Agents | New capability |
|---|---|---|---|---|
| **A** | Fixed-time signal | No | 1+ | Vehicles learn to read phase + timing under latency and loss |
| **B** | Actuated — phase adapts to announced intent and queue | Yes | ≥2 | Uplink earns its existence; contention is real |
| **C** | Learned arbiter — node optimizes throughput | Yes | ≥2 | Infrastructure as an RL agent |

Stage A is meaningful with a single vehicle: a fixed-time signal blocks
regardless of who else is present, so the learning problem is real. This
contrasts with FCFS arbitration, where a lone agent is always queue head and
always receives GO — a single-agent test of that path exercises nothing.

---

## 10. J2735 correspondence

Documented for reviewers; no dependency is implied.

| This protocol | J2735 nearest equivalent |
|---|---|
| `IntentAnnouncement` | `BasicSafetyMessage` + `RequestedItem` (SRM intent extension) |
| `MovementAdvisory.phase_*` | `SPAT.movementState.eventState` |
| `MovementAdvisory.t_to_change` | `SPAT.movementState.timing.minEndTime` |
| `MovementAdvisory.conflict_ttc` | No direct equivalent; inferred from received `BSM` set |
| `MovementAdvisory.slot_open/dur` | No equivalent; nearest is SSM reservation semantics |
| `age` / `valid` | No equivalent — J2735 has no in-band staleness field |

The last row is the substantive divergence. Explicit staleness is what allows
the policy to learn a fallback behaviour rather than silently acting on
expired data.

---

## 11. Open items

1. **Per-neighbour encoding.** Fields 7–8 are aggregates. Gap acceptance and
   merging need per-neighbour state, which implies variable-width input and an
   attention or DeepSets encoder. Deferred to Stage C; if merging becomes part
   of the claim, this decision must be revisited *before* the retrain, not
   after.
2. **Sim-time source.** `SchedulerCore` currently uses `time.monotonic()` and
   a 15 s `intent_timeout`. Both must move to sim time
   (`episode_length_buf * dt`) during the port.
3. **Reward integration.** Introduce exactly one new term initially — a
   red-light violation penalty — with the throttle shield still active. The
   shield's activations should be counted and penalized so the policy learns
   not to need it, then annealed. Adding a shaping suite up front repeats the
   failure mode documented as Issues 10–13.
4. **Normalization constants.** `D_MAX` is now pinned to the resolved
   `junction_18` geometry (§3.2). `T_MAX` and `N_MAX` remain provisional
   pending the first training run.
5. **Stale 8× annotation metadata.** The `sub_intersections/network` subtree in
   the USD was not rescaled in Phase 14-01. Anything reading it must divide by
   8. Worth either rescaling the subtree or deleting it, since it is a standing
   trap — it already produced one wrong constant.
6. **Channel constants are superseded, not merely provisional.** See the note
   in §6: they predate §3.2 and the timescale does not close at F1Tenth scale.
   Re-deriving them is a prerequisite to implementing the ring buffer, and the
   right values depend on the sim-only vs. sim-to-real scope decision, which is
   still open.

---

## 12. Wire format (deployment)

For hardware, the float vectors above serialize to ROS2 message definitions —
plain text, human-readable, no ASN.1:

```
# MovementAdvisory.msg
uint8   proto_version
uint8   phase              # 0=permitted 1=changing 2=stopped
float32 t_to_change        # seconds, unnormalized
float32 conflict_ttc       # seconds, unnormalized; inf if no conflict
uint8   queue_ahead
float32 conflict_load
float32 slot_open
float32 slot_dur
```

Normalization is applied at the observation boundary, not on the wire — the
wire carries SI units so the messages remain independently meaningful.
