# Communications Branch — Session State

**Branch:** `aaron/communications` (off `main` @ `2b9b49d`)
**Date:** 2026-07-27
**Task:** Design + implement V2I message passing between vehicles and intersection infrastructure nodes.

---

## 1. Status

`docs/V2I_PROTOCOL.md` v0.2.0 is the deliverable. Committed as `a760312`
(v0.1.0), revised to v0.2.0 in this session. **No source changes yet.**

Every factual claim in both documents has been traced to code on post-SKRL
`main` and corrected where it disagreed. Two findings were retracted (§4.1) and
one bug closed as already-fixed upstream (B6). Three integration conflicts with
`main` were found and are recorded in §4.2 — they are the reason V2I code is
Phase 2, not now.

**Design intent, one paragraph.** The radio carries only what a camera cannot
recover (spec §2.1); the vehicle *announces* intent and the node *arbitrates*
(`scheduler_core.register_intent` returns go/wait, never an assigned turn); the
message set encodes no fleet-size constant, so 8 vehicles is a deployment number
rather than an architectural one. The first principle is Arika's, not ours —
`go_signal_manager.py`'s docstring states it for a single field ("the same module
will run on the real D435i camera stream"), and the spec generalizes it to the
whole message set. Where the design and post-SKRL `main` disagree, `main` wins:
see §4.2.

`.planning/RESUME.md` shows as modified and **must not be committed** — see §6.

---

## 2. Environment facts

- This Mac is an **edit-only checkout**. No Isaac Lab, no `tmux`, no `logs/`, no
  `python` on PATH; `python3` exists but has no numpy. Training runs on a
  separate Linux box (`/home/arika/IsaacLab`).
- Anything needing a runtime check must be run there.
- `.usda` files are ASCII and greppable locally — this is how §3.2 of the
  protocol was resolved.

---

## 3. Bugs found

All traced against post-SKRL `main` on 2026-07-27. None fixed yet — this pass is
documentation only. Fix order is §7; B4 is first.

**B1 — `waypoint_progress_reward` accumulates instead of differencing.**
`arcproLab/mdp/rewards.py:70-76`. `prev_wp_idx_reward` is updated only when
`reset_buf` is True, so it never advances during normal driving. `delta`
therefore measures progress since the last *reset*, not the last *step*, making
episode return quadratic in length. Explains the `WPΔ_rew` spikes (2434/7300/5795)
logged as "Issue 16" — they appeared at 640k-716k steps only because the physics
fixes let episodes survive long enough for the accumulation to show. Fix: always
store `current_idx`/`current_pos`; keep the `reset_buf` gate for zeroing output;
carry the reset mask forward one step (the bogus wrap lands the step *after*
reset, since rewards compute before the teleport); add a plausibility clamp.

**B2 — Asymmetric actor-critic is inverted.**
`arcproLab/mdp/observations.py:147-149`. `lat_err`/`head_err` are written only
under `if masked:` — that branch is the *policy*. The critic (`masked=False`)
receives zeros in slots 8-9, while the actor additionally gets 512 vision dims.
The "privileged" critic sees a strict subset of the actor. Inverts Pinto et al.
Plausibly contributes to the value-function pathologies in Issues 11-13.

**B3 — Critic input shape unverified.**
`arcproLab/agents/skrl_models.py:46` hardcodes `nn.Linear(12, 256)`; the
recovery branch at :71-72 tests for `150540`, the dead pre-ResNet width. Wrapper
advertises 524. Whether this crashes or silently mis-feeds depends on whether
the installed SKRL routes `env.state()` to the value model. **Check on the
training box:** print `inputs["states"].shape` inside `ARCProCritic.compute`.

**B4 — `stop_line_detector.py` is missing from `main`.** *(highest priority)*
`arcproLab/mdp/go_signal_manager.py` imports it; the file exists only on
`origin/Aaron_Summer_Testing_V1` (439 lines) and in the policy repo (505 lines,
newer). Consumers that would raise swallow it — `observations.py:74-77` (bare
except → `obs[:,1] = 1.0`, always GO), `actions.py:215-220` (`except: pass`),
`events.py:111`. `arcpro_env_cfg.py:191` never sees the error at all: it reads
`env.extras.get("go_signal", torch.ones(...))`, so the default silently supplies
GO. **`go_signal` is permanently 1.0 and nothing logs it.**

Consequence beyond the obvious: this is *Arika's* intended perception path
(`go_signal_manager.py` docstring — the same module runs on the real D435i
stream), so its absence disables the one deployment-realistic infrastructure
channel `main` has. `docs/V2I_PROTOCOL.md` §5.2 and §8 both route through it.
Restoring it works *with* her architecture, which is why it precedes B1/B2.

**B5 — `road_manager.py` is dead on `main`.** `go_signals` hardcoded to 1.0 at
:87 and discarded by the caller (`observations.py:41` does `turn_tokens, _ =`).
`gates_pos`/`gates_intent` populated by the USD scan at :33-67, never read.
**Not a delete candidate** — it is the splice target and integration point on
`aaron/v2i_intersection` (+41 lines there, applied by `tools/apply_v2i_splice.py`).
It is dead on `main` because the branch that animates it was never merged.

**B6 — ~~distance accumulator integrates at the wrong dt~~ — ALREADY FIXED on
`main`.** The defect (0.05 s against a 0.02 s control step, slot 11 over-reading
2.5×) was real when `V2I_DESIGN.md` §7 recorded it. `observations.py:137-138`
now reads `# FIX: Use 0.02 for 50Hz clock (was previously 0.05 for 20Hz)`.
Carried into an earlier draft of this file from `V2I_DESIGN.md` without
re-checking the code. **No action.** Kept here as the record of a stale finding,
since `V2I_DESIGN.md` on the branch still lists it as open.

**B7 — action-slot mismatch.** *(live)* Obs slots 5–7 expect 3 actions;
`ActionCfg` defines 2. Silent no-op, one slot permanently zero. Same source as
B6 — verified still present.

B6 and B7 were tracked in `V2I_DESIGN.md` §7 toward a `CONTRACT_MISMATCHES.md`
that **exists in neither repo**. This file is now their home. That B6 had since
been fixed upstream without the branch doc learning of it is the exact staleness
pattern this consolidation exists to stop.

---

## 4. Repo decisions

**`arc_rl_isacc_policy` is stale and superseded** (user's call). Its V2I stack
is good but targets SB3 + Gymnasium over a Direct env; sim moved to SKRL +
IsaacLab `ManagerBasedRLEnv`. V2I ownership **moves into `arc_rl_isacc_sim`**.

Worth porting (architecture-neutral, ~1500 lines, already tested):
`scheduler_core.py`, `scheduler_transport.py`, `intersection_node_server.py`,
`intersection_graph.py`, `intersection_geometry.py`, `stop_line_detector.py`,
`config/intersection_topology.json`, `tests/`.

Must be rewritten (targets the abandoned stack): `agent_env_wrapper.py` and the
`WorkerScheduler` facade.

Known defects in the ported code: `SchedulerCore` uses `time.monotonic()` and a
15 s `intent_timeout` (must become sim time; 15 s is also 31× the entire
approach window); `IntentMessage` carries world-frame position/heading (must
become ego-relative); scalar Python dicts keyed by string agent ID (must become
tensors).

### 4.1 `aaron/v2i_intersection` — keep/port verdict

Fork point `c6c4d3c` (2026-06-20), **before** the SKRL merge. Needs rebasing
onto post-SKRL `main` before anything ports. 5 modules, 36 CPU tests, 2 splice
tools.

| File | Verdict |
|---|---|
| `mdp/v2i_buffer.py` | **Keep, extend.** The channel model. Written, tested, not wired into the loop |
| `mdp/signalized_scheduler.py` | **Keep.** See retraction below |
| `mdp/intersection_manager.py` | **Keep.** Batched multi-phase FSM, `(B,N)`-ready |
| `mdp/intersection_rewards.py` | **Keep.** Gated terms, extras-driven, behavior-neutral until `go_signal` publishes |
| `mdp/stoplight_visual.py` | **Keep.** Per-env emissive prims |
| `V2I_DESIGN.md` | **Keep, revise.** Prior art; `docs/V2I_PROTOCOL.md` §0 now defers to it |

**Retraction (2026-07-27).** An earlier pass in this session marked
`signalized_scheduler.py` and `intersection_manager.py` as superseded on the
grounds that the FSM *assigns* turns, contradicting the vehicle-announces-intent
requirement. That was wrong. The docstring is explicit: *"`register_intent`
doubles as the actuation uplink: a registered intent is standing demand for its
approach group… A vehicle that registers is collaborating; one that never
registers still gets correct (fixed-time) signals."* That is
vehicle-announces-intent with graceful uplink degradation. The verdict came from
reading the commit message (`62e6e1d`, "oracle comms, single-agent") rather than
the code.

**Delete candidates in sim repo:** `archive/sb3_legacy/policy_stack/` (a stale
*divergent fork* of the policy repo — all 5 overlapping files differ, e.g.
`fusion_policy.py` by 314 lines), `mdp/observations_with_lat_err.py` (zero
importers), `.planning/CONTEXT_CHECKPOINT.md` (stale, pre-SKRL). All three
verified present.

**Second retraction (2026-07-27).** An earlier draft listed
`mdp/go_signal_manager.py` as "superseded by the branch's scheduler stack."
Wrong, and in the more damaging direction. It is `main`'s **primary
perception-derived infrastructure channel** and it stays. Its docstring states
the deployment argument directly: the detector is privileged in sim but *"the
same module will run on the real D435i camera stream… so unlike lat_err/head_err
(oracle geometric truth, PVP-masked), go_signal IS shown to the policy."* The
branch's scheduler produces an oracle traffic-light phase — a different quantity
with different provenance, not a replacement. See §4.2.

### 4.2 Integration conflicts with post-SKRL `main` (blocking)

Found by tracing `main`'s code and decision register after a cherry-pick of
`aaron/v2i_intersection` was started and aborted. All three would have shipped
silently.

| # | `main` (Arika) | `aaron/v2i_intersection` | Resolution |
|---|---|---|---|
| C-1 | Telemetry slot 1 = stop-**sign** dwell FSM, camera-derived (`go_signal_manager.py`, T3.2/T3.3) | slot 1 = traffic-**light** phase, oracle sim state | **Branch loses the slot.** Feeding oracle state to the actor is D010's failure mode and violates C7. Slot 1 keeps perception semantics; phase arrives as advisory fields 2–4. See `docs/V2I_PROTOCOL.md` §5.2 |
| C-2 | Phase 1 curriculum: `lateral_error`, `heading`, `smoothness`, `jerk` all weight **0.0** by design; `STATE.md` Next Action still targets `ep_len_mean ≥ 600`, `speed > 0.5`, `WPΔ > 0` | adds `red_light` (w100) + `hold_at_red` (w10) | **Defer to Phase 2.** D018 records the ResNet brain hitting immediate termination when Phase 2 penalties land early. Landing a w100 penalty during Phase 1 repeats it |
| C-3 | `speed_reward` **deleted**; replaced by `waypoint_progress_reward` (w20) + `action_drive_reward` (w0.5) | `gated_speed_reward` wraps `mdp_rew.speed_reward` | **Cannot port as written** — the wrapped function no longer exists, and D022 is the register's only `Revisable? No` row. Re-target onto `waypoint_progress_reward` or drop |

The cherry-pick was aborted (`git cherry-pick --abort`); tree verified identical
to `main` at `a760312`, no conflict markers, no local v2i branch,
`origin/aaron/v2i_intersection` untouched.

Note on C-2's mechanism: an earlier proposal to "gate `progress_reward` on
`go_signal`" was incoherent and is withdrawn. `progress_reward` is a *positive*
term — gating it while the vehicle is stopped is a no-op. The real mechanism is
opportunity cost, which `hold_at_red` already supplies. `stationary` on `main`
(`arcpro_env_cfg.py`) additionally gates its penalty on `go_signal > 0.5`
inline, so a stopped vehicle at a red bar is already not punished.

---

## 5. Design decisions

1. **Message-passing peer**, not privileged oracle.
2. **Vehicle announces intent**; the node arbitrates around it.
3. **Custom protocol, not SAE J2735** — paywalled, ASN.1, wrong scale. A
   correspondence table is kept in spec §10.
4. **Radio carries only what vision cannot obtain** (spec §2.1, constraint C7).
   Derived from Arika's D010 (reverted: waypoints in *observation* bypass R002)
   vs D022 (accepted: same waypoints in *reward*). The governing rule —
   *privileged info may shape reward or inform the critic, never the actor's
   observation* — is the asymmetric actor-critic contract stated informally.
5. **`d_to_stopline` removed from the downlink**; it is visually determinable
   and already computed by `VisualStopLineDetector`.
6. **Message width frozen** at 12 downlink / 8 uplink floats.
7. **The uplink costs zero observation width.** Nothing announced is a policy
   choice — turn intent is the existing `RoadManager` mission token, and
   position/heading/speed are state the env already holds. The announcement is
   assembled env-side, consumed node-side, never entering the actor's
   observation. Adding it forces no retrain.
8. **8 vehicles, but N is not a design parameter** (spec C8). All cars run **one
   shared policy** — identical weights, own observation. N is a batch dimension.
   MARL later is a change of training scheme, not of message format.
9. **`N_MAX` deleted.** It was a capacity constant; §3.1 says changing a
   normalization constant invalidates every checkpoint, so it would detonate at
   the 9th vehicle. `queue_ahead` now uses saturating `q / (q + 4)`.
10. **Neighbour state is a fixed-width pooled block**, not per-neighbour slots.
    Aggregate encoding *is* pooling with a hand-designed φ; DeepSets is the same
    operation with a learned φ. Freezing the pooled *width* rather than a
    neighbour count means the swap costs no retrain. Closes what was spec §11
    item 1.
11. **Policy stays feedforward.** No LSTM. Held-value drift is closed-form and
    is dead-reckoned inside the buffer (spec §6.3); recurrence's best payoff —
    temporal vision — is foreclosed by the frozen backbone; hidden state is a
    sim-to-real hazard. Observation stacking is the fallback if the policy
    plateaus.
12. **Channel parameters are randomized per episode, not fixed.** Sim-to-real is
    in scope, so the measured radio profile must land *inside* a trained range
    rather than match a number. `MAX_AGE_STEPS` stays fixed — it normalizes an
    observation field.
13. **Stage C (learned arbiter / infrastructure as RL agent) cut.** No code, and
    it was the sole justification for two message fields and for the
    "allocate width now" argument. Demoted to a one-line future direction.

**No `DECISIONS.md` row appended.** §2.1's rule is Arika's, not ours — recording
it would misattribute existing doctrine. Items 2, 3, 8, 10 and 11 are genuinely
new and warrant rows, but the register records decisions *acted on* and the spec
is still `Status: Proposed`. Append when implementation lands.

---

## 6. Stale artifacts to remove

Present goal: **delete documentation and ideas not reflected in present code.**

| Target | Why |
|---|---|
| `V2I_DESIGN.md` Layer 4 RecurrentPPO | Code archived under `sb3_legacy`; D017 migrated to feedforward SKRL PPO |
| `V2I_DESIGN.md` §7 → `CONTRACT_MISMATCHES.md` | File never existed; repoint at §3 above |
| `V2I_DESIGN.md` §2 "10 Hz → `beacon_interval=5`" | Automotive scale; see spec §6.1 |
| `V2I_DESIGN.md` §4.3 valid/age "open sync item" | Resolved — spec §5 fields 0–1 |
| `v2i_buffer.py` docstring 10 Hz anchor | Same error as above, in shipped code |
| `.planning/INTERSECTION_NODE_DESIGN.md` citations | Never committed anywhere; 3+ policy-repo files cite it |
| `.gsd/milestones/M001/M001-ROADMAP.md` Boundary Map | Still lists SB3 + Gymnasium; D017 migrated to SKRL |
| `archive/sb3_legacy/policy_stack/` | Stale divergent fork |
| `mdp/observations_with_lat_err.py` | Zero importers |
| `.planning/CONTEXT_CHECKPOINT.md` | Pre-SKRL |
| USD `sub_intersections/network` 8× subtree | Stale metadata in a 1× file; already produced one wrong constant |
| Policy repo `agent/README.md` | Documents `topological_ekf.py` (deleted in `012c716`); describes the scheduler as decentralized/ROS2-broadcast, the opposite of the current centralized `IntersectionNodeServer` |

Also: `.gsd/KNOWLEDGE.md` Rules and Patterns tables are **empty**. The
provenance rule (spec §2.1) belongs there — it currently exists only as an
inference across two decision rows nine entries apart, which is why it was
nearly missed.

**`.planning/RESUME.md`** is a 2-line stub reading "read RESUME.md instead" —
macOS case-folds it with `.planning/resume.md`, so it points at itself. Git
tracks both casings; committing the file deletes Arika's real 77-line context
doc. Real content is at `git show HEAD:.planning/RESUME.md`. **Fix on the Linux
box** with `git rm --cached` on one casing.

---

## 7. Next actions

Scope questions are resolved: **sim-only now, sim-to-real to physical hardware
after successful training. 8 vehicles, one shared policy.**

Ordered so that every step works *with* post-SKRL `main` rather than against it
(§4.2). Phase 1 must stay closable before any V2I reward lands.

**Now — correctness on `main`, no V2I code**

1. ~~Commit the spec.~~ Done — `a760312`, revised to v0.2.0.
2. ~~Correct both docs against traced code.~~ This pass. Commit **docs only**;
   `.planning/RESUME.md` stays unstaged (§6).
3. **B4 — restore `stop_line_detector.py`.** Port the 505-line policy-repo
   version. Highest value and lowest conflict: it resurrects Arika's intended
   perception path, un-pins telemetry slot 1 from 1.0, and makes §5.2/§8 of the
   spec testable for the first time. Add logging for `go_signal` — its silence
   is why the defect survived.
4. **B1 — `waypoint_progress_reward` differencing.** It is the Phase 1 reward
   (w20) and it is quadratic in episode length; every Phase 1 exit criterion is
   read through it.
5. **B2 — un-invert the asymmetric critic**, carrying B7 along. One retrain, not
   two.
6. **B3 — verify critic input shape on the training box** before that retrain.

**Then — Phase 1 closes, V2I becomes Phase 2**

7. **Revise `V2I_DESIGN.md`** on its branch — items in §6 above, plus B6 (fixed
   upstream, still listed open there).
8. **Rebase `aaron/v2i_intersection`** onto post-SKRL `main`, resolving C-1/C-2/
   C-3 (§4.2) at rebase time. Nothing ports until this happens.
9. **Wire `v2i_buffer.py` into the loop** and add the per-field schema for
   dead-reckoning (spec §6.3). It exists and is tested; it was never connected.
10. **Fix the N=1 observation mapping** (spec §11 item 4). `RoadManager` and the
    FSM are `(B,N)`-ready; the obs mapping is not. Blocking for 8 vehicles.
11. Introduce **one** V2I reward term (red-light violation) with the throttle
    shield active, per spec §11 item 2. Not a suite — that is C-2/D018.
12. Confirm `T_MAX` against a first run. `D_MAX` is pinned (spec §3.2).
13. Append `DECISIONS.md` rows when implementation lands.

**Note on "real-time":** there is none during training. Isaac Sim is one
process in lockstep with the physics tick — no sockets, no clock, no
concurrency. Real-time exists only at deployment, and the channel model is the
sole bridge to it. That is why its constants matter more than they look.
