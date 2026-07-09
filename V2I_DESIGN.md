# V2I smart intersection — design document

Branch: `aaron/v2i_intersection` · Author: Aaron · Updated: 2026-07-09
Status: implemented and CPU-tested (36 tests); GPU validation pending
(GUI verify → 1-env smoke test → stop-and-go training run).

## 1. Vision

Fully autonomous vehicles collaborating with an intersection node to form
decisions under uncertainty. The design locates uncertainty in the two
channels between vehicle and infrastructure — the radio (latency, dropout,
staleness) and the camera (occlusion, range, lighting) — rather than in
the endpoints. Both endpoints make decisions; the channels degrade. The
policy learns how much to trust each channel from that channel's own
declared confidence, rather than from a hand-coded fusion rule.

## 2. Layered architecture

The system separates four concerns; no layer reaches around another.

**Layer 1 — Arbitration authority (the intersection's brain).** Pluggable
behind one interface, the policy repo's SchedulerCore-shaped API:
`register_intent / query_go_signal / clear_agent / tick / active_intents`.
Two backends exist: the policy repo's `IntersectionNodeServer`
(reservation-based, conflict-matrix grants) and this branch's
`SignalizedScheduler` (phase-based traffic light, wrapping the FSM). A
learned scheduler (intersection as RL agent) is a future drop-in. Because
both backends emit the same message shape, signalized vs. reservation
control are directly comparable under identical conditions.

**Layer 2 — Transport (V2I uncertainty injection).** `V2IMessageBuffer`
models the radio: `latency_steps`, `dropout_p`, `beacon_interval`,
`stale_after`. Ideal comms is the degenerate config (0/0/1/None), so
realism is a knob and never a refactor. The buffer returns
`(held, valid, age)` — comms failure is an *observable*, which is the
precondition for the policy reasoning about it. Real-world anchor: 10 Hz
SAE J2735-style beaconing = `beacon_interval=5` at the 50 Hz control rate.

**Layer 3 — Perception (the redundant physical channel).**
`StoplightVisual` builds one emissive stoplight per env clone beside its
first laneGate; lamp visibility tracks signal phase. This channel exists
only under signalized authority — the asymmetry (reservation grants have
no pixels) is a subject of study, not a bug. Combined with the vision
mandate (obs slots 8–9 masked to force the ResNet to learn from pixels),
it enables the redundancy experiment: degrade the radio, watch the policy
shift weight to the light it can see.

**Layer 4 — Decision (the vehicle).** RecurrentPPO fusing ResNet-18
features with the 12-dim telemetry vector. Recurrence bridges dropout gaps
("the light was red 400 ms ago and horizon said 4 s; it is almost
certainly still red").

## 3. Component inventory

| File | Role | Tests |
|---|---|---|
| `arcproLab/mdp/intersection_manager.py` | Batched multi-phase signal FSM: N approach groups, per-group green, phase randomization on reset, capped actuated extension | 6 |
| `arcproLab/mdp/v2i_buffer.py` | Latency / dropout / beacon-rate / staleness transport model | 6 |
| `arcproLab/mdp/intersection_rewards.py` | Gated reward terms, extras-driven, behavior-neutral until go_signal is published | 8 |
| `arcproLab/mdp/signalized_scheduler.py` | FSM behind the SchedulerCore-shaped API (duck-typed); register_intent = actuation uplink | 7 |
| `arcproLab/mdp/stoplight_visual.py` | Per-env emissive stoplight prims; visibility synced on phase transitions only | 5 |
| `arcproLab/mdp/road_manager.py` (spliced) | Integration point: ticks FSM, drives go_signals/ttc, builds stoplights at gate discovery, publishes extras | 4 |
| `tools/apply_v2i_splice.py`, `tools/apply_stoplight_splice.py` | Anchored patch scripts; abort loudly on mismatch, nothing written on failure | — |

Run all: `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 isaaclab.sh -p -m pytest tests/ -q`
(autoload disabled because ROS Jazzy injects an incompatible py3.12 pytest
plugin into kit py3.11 — see `tests/run_tests.sh`).

## 4. Contracts

### 4.1 Observation protocol (12-dim telemetry, unchanged shape)

| Slot | Meaning | Source |
|---|---|---|
| 0 | turn token (−1 left / 0 straight / 1 right) | RoadManager missions (pre-existing) |
| 1 | go_signal (1 proceed / 0 stop; yellow = stop) | IntersectionManager via RoadManager |
| 2 | permission horizon, normalized by cycle_s | IntersectionManager (was zeroed IDX_GOAL_DIST) |
| 3–11 | speed, yaw rate, last actions, masked errors, kappa, distance | unchanged |

The splice implements the slot semantics main already declared (slot 1 was
go_signal hardcoded to 1.0; slot 2 was reserved). No policy-side shape
renegotiation was required. A validity/age slot is deliberately deferred:
phase A runs oracle comms, where validity is constant.

### 4.2 Extras contract

Reward terms read ONLY `env.extras` — never the RoadManager singleton:
`extras["go_signal"]` (published by observations post-splice) and
`extras["dist_g"]` (pre-existing, unsigned min-Euclidean to nearest gate).
Missing keys default to all-green / far, so wiring the reward terms into
RewardCfg was behavior-neutral before the splice. The splice is therefore
the single activation switch for the entire feature; reverting is one flag
(`RoadManager.intersection = None` restores legacy always-green).

### 4.3 Unified V2I message (proposed, pending sync)

`permission` (go/stop) · `horizon` (ttc for a light, grant TTL for a
reservation) · `phase` (optional) · `valid` · `age` (transport-added).
One message shape for both authorities is what makes the paradigm
comparison fair. Slot placement of valid/age is an open sync item.

### 4.4 Scheduler API conformance notes

`SignalizedScheduler` duck-types rather than subclasses `SchedulerCore`:
the class lives on policy main while the sim's submodule pin (`adda709`)
is on an older lineage, and the adapter must not depend on which commit is
checked out. `register_intent` returns the permission horizon — whether
SchedulerCore's contract is ETA or ack needs confirmation (sync item).
Fail-safe semantics throughout: unknown agents and yellow query as stop.

## 5. Reward design

Zone-based, because `dist_g` is unsigned and the nearest gate switches
after a crossing (sign-flip detection would be unreliable): a violation is
motion above `v_stop` inside `violation_radius` on red. The two legacy
terms that actively punished stopping (`speed` w20, `stationary` w5) are
gated off inside the red zone; `hold_at_red` (w10) makes stopping
positively attractive rather than merely un-penalized; `red_light` (w100)
penalizes running the line. A `DoneTerm` variant exists but is
deliberately not enabled — phase it in only if the policy plateaus while
still running lights. Weights are first-guess and expected to need tuning
against the first real run.

## 6. Validation plan (GPU, in order)

1. **GUI verify**: stoplights standing beside gates, lamps cycling on the
   20 s cycle, pole clear of the driving line (`LATERAL_OFFSET` tunable).
2. **1-env headless smoke test** (~2000 timesteps): survives past Isaac
   startup, SB3 tables print, `hold_at_red` / `red_light` appear in the
   reward breakdown with `hold_at_red` occasionally nonzero.
3. **Stop-and-go training run** (overnight, 1–4 envs): agent learns to
   stop on red, proceed on green. Watch for reward-scale pathologies
   (loitering outside the 3 m approach radius; red_light weight vs.
   episode return).

## 7. Known limitations and deliberate deferrals

Single agent (obs mapping assumes N=1; RoadManager and FSM are (B,N)-ready).
One intersection per env (FSM has no intersection dimension yet;
multi-intersection needs agent→intersection assignment, which is the
policy repo's `intersection_graph` territory — convergence point, not
unilateral work). Approach groups are all-zeros pending route-derived
assignment. Oracle comms in the training path (buffer exists, not yet in
the loop). Global gate list across env clones (correct while env spacing
exceeds gate spacing; per-env indexing is the known fix). `_ROAD_MANAGER`
singleton ignores constructor args after first call. Known pre-existing
bugs, tracked for CONTRACT_MISMATCHES.md: distance accumulator integrates
at 0.05 s against a 0.02 s control step (slot 11 ≈2.5× overestimate); obs
slots 5–7 expect 3 actions while ActionCfg defines 2 (silent no-op).

## 8. Open items for the Arika sync

Push the `adda709` policy-repo lineage (fresh clones cannot train without
it). Confirm SchedulerCore return semantics and API stability. Agree the
unified message contract and valid/age slot placement. Align intersection
ownership around the pluggable-authority architecture (this branch adopts
the policy repo's API as the spine; reward-term merger into
`intersection_reward_wrapper` expected). Scope the research question —
signalized vs. reservation under matched comms degradation — jointly for
Golnaz.
