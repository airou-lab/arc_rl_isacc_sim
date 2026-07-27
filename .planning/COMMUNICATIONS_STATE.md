# Communications Branch — Session State

**Branch:** `communications` (off `main` @ `2b9b49d`, verified current with `origin/main`)
**Date:** 2026-07-27
**Task:** Design + implement V2I message passing between vehicles and intersection infrastructure nodes.

---

## 1. Working tree (uncommitted)

| File | State |
|---|---|
| `docs/V2I_PROTOCOL.md` | **New.** The deliverable. Full protocol spec v0.1.0 |
| `.planning/COMMUNICATIONS_STATE.md` | **New.** This file |
| `.planning/RESUME.md` | Modified — pre-existing artifact, not ours. See §6 |

Nothing committed yet.

---

## 2. Environment facts

- This Mac is an **edit-only checkout**. No Isaac Lab, no `tmux`, no `logs/`, no `python` on PATH; `python3` exists but has no numpy. Training runs on a separate Linux box (`/home/arika/IsaacLab`).
- Anything needing a runtime check must be run there.
- `.usda` files are ASCII and greppable locally — this is how §3.2 of the protocol was resolved.

---

## 3. Bugs found on `main` (reviewed, NOT fixed — user directed review-only)

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

**B4 — `stop_line_detector.py` is missing from `main`.**
`arcproLab/mdp/go_signal_manager.py` imports it; the file exists only on
`origin/Aaron_Summer_Testing_V1` (439 lines) and in the policy repo (505 lines,
newer). Every consumer swallows the ImportError — `observations.py:74-77` (bare
except → always GO), `actions.py:215-220` (`except: pass`), `events.py:111`,
`arcpro_env_cfg.py:191`. **`go_signal` is permanently 1.0 and nothing logs it.**

**B5 — `road_manager.py` is largely dead.** `go_signals` hardcoded to 1.0 at
:87 and discarded by the caller (`observations.py:41` does `turn_tokens, _ =`).
`gates_pos`/`gates_intent` populated by the USD scan at :33-67, never read.

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
15 s `intent_timeout` (must become sim time); `IntentMessage` carries world-frame
position/heading (must become ego-relative); scalar Python dicts keyed by string
agent ID (must become tensors).

**Delete candidates in sim repo:** `archive/sb3_legacy/policy_stack/` (a stale
*divergent fork* of the policy repo — all 5 overlapping files differ, e.g.
`fusion_policy.py` by 314 lines), `mdp/observations_with_lat_err.py` (zero
importers), `mdp/road_manager.py`, `mdp/go_signal_manager.py`,
`.planning/CONTEXT_CHECKPOINT.md` (stale, pre-SKRL).

---

## 5. Design decisions this session

1. **Message-passing peer**, not privileged oracle.
2. **Vehicle announces intent**; the node arbitrates around it. Diverges from
   `road_manager.py`, where turns are assigned.
3. **Custom protocol, not SAE J2735** — paywalled, ASN.1, wrong scale. A
   correspondence table is kept in spec §10.
4. **Radio carries only what vision cannot obtain** (spec §2.1, constraint C7).
   Derived from Arika's D010 (reverted: waypoints in *observation* bypass R002)
   vs D022 (accepted: same waypoints in *reward*). The governing rule —
   *privileged info may shape reward or inform the critic, never the actor's
   observation* — is the asymmetric actor-critic contract stated informally.
5. **`d_to_stopline` removed from the downlink**; it is visually determinable
   and already computed by `VisualStopLineDetector`. Slot 6 now carries
   `conflict_ttc` (occluded conflicting vehicle) which is genuinely non-visual.
6. **Message width frozen** at 12 downlink / 8 uplink floats. Stage A uses 7;
   the rest transmit zero. Avoids a second retrain at Stage B.

**No `DECISIONS.md` row appended.** §2.1's rule is Arika's, not ours — recording
it would misattribute existing doctrine. Items 2 and 3 above are genuinely new
and warrant one row, but the register records decisions *acted on* and the spec
is still `Status: Proposed`. Append when implementation lands.

---

## 6. Other stale things found on `main`

- `.planning/RESUME.md` is a 2-line stub reading "read RESUME.md instead" — macOS
  case-folds it with `.planning/resume.md`, so it points at itself. Real content
  is at `git show HEAD:.planning/RESUME.md`.
- `.gsd/milestones/M001/M001-ROADMAP.md` Boundary Map still lists **SB3 and
  Gymnasium**; D017 migrated to SKRL and it was never updated.
- `.gsd/KNOWLEDGE.md` Rules and Patterns tables are **empty**. The provenance
  rule (§5.4) belongs there — it currently exists only as an inference across two
  decision rows nine entries apart, which is why it was nearly missed.
- USD `sub_intersections/network` subtree is **stale 8× metadata** in a 1× file.
  Divide by 8. Already produced one wrong constant.
- Policy repo's `agent/README.md` documents `topological_ekf.py` (deleted in
  `012c716`) and describes the scheduler as decentralized/ROS2-broadcast, which
  is the opposite of the current centralized `IntersectionNodeServer`.
- `.planning/INTERSECTION_NODE_DESIGN.md` is cited as authoritative by 3+ files
  in the policy repo and **exists in neither repo**. `docs/V2I_PROTOCOL.md`
  supersedes it.

---

## 7. Next actions

**Blocked on one decision from Aaron: is sim-to-real transfer to physical
F1Tenth hardware in scope, or is the claim sim-only?** It sets the channel
constants (item 2) and therefore gates implementation.

1. **Commit the spec** to `communications`.
2. **Re-derive the channel constants from §3.2 geometry.** The drafted values
   are *wrong*, not merely provisional — they predate the geometry resolution.
   At 1.5 m/s the spawn→stop-line approach is ~480 ms, but `max_age_steps = 25`
   is 500 ms, so an advisory outlives the whole approach. Expect ~50 Hz
   broadcast, 1–2 step latency, `max_age_steps` ≈ 5. Flagged in spec §6 and
   §11 item 6.
3. **Settle aggregate vs per-neighbour encoding** (spec §11 item 1). Fixes the
   message width, so it must precede any training — getting it wrong costs a
   full retrain.
4. **Build the channel model** — self-contained tensor work, testable on the
   Mac without Isaac Sim, and both actor and critic paths hang off it. Ring
   buffer `(num_envs, num_agents, horizon, D_msg)` + `delivery_step`.
5. Confirm `T_MAX` / `N_MAX` against a first run (`D_MAX` is pinned — §3.2).
6. Fix B2 and widen the observation in **one** retrain, not two.

**Note on "real-time":** there is none during training. Isaac Sim is one
process in lockstep with the physics tick — no sockets, no clock, no
concurrency. Real-time exists only at deployment, and the channel model is the
sole bridge to it. That is why its constants matter more than they look.
