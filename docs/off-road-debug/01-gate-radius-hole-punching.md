# Fix: gate-zone radius / boundary hole-punching mismatch

**Branch:** `off-road-debug` (off `main` @ c6c4d3c)
**Files touched:** `arcproLab/mdp/track_manager.py`, `arcproLab/mdp/terminations.py`
**Status:** Implemented, not yet verified in a live rollout (see Verification below)

## Problem this addresses

Aaron and Arika observed the car driving off the road past the white
boundary lines during training, and suspected the agent (or the
environment's own boundary logic) was conflating the white road-edge lines
with the white stop-line paint at intersection gates. This doc covers the
first confirmed root cause in that area.

## Root cause

The environment has two independent copies of "how close to a gate counts
as being in the gate," and they drifted apart from the documented design
value without anyone updating the design doc or explaining the change:

| Consumer | Purpose | Documented spec | Code before this fix | Introduced at |
|---|---|---|---|---|
| `terminations.py::white_line_contact` (`in_gate_zone`) | Suppresses white/yellow-line termination when the robot is "in" a gate | `dist_gate < 0.20m` | `dist_g < 0.50` | Regressed in `0a97fb9` |
| `track_manager.py::collect_raw_marker_points` (`punch_holes`) | Deletes white/yellow boundary points near a gate from the point cloud used to compute `dist_w`/`dist_y` | — (never specified) | `dists > 1.0` | `2d28a8b` at 0.5m, widened to 1.0m in `0a97fb9` |

Design spec source: `.planning/research/15-MASTERY_REWARDS.md` ("Immunity:
If $dist\_gate < 0.20m$, set $R_{line} = 0.0$"), echoed in
`.planning/phases/15-mastery-curriculum/15-01-PLAN.md` and
`15-01-SUMMARY.md" ("Reset is suppressed if the robot is within 0.20m of a
gate zone"). This 0.20m value was correctly implemented in commit `83e6818`
(Phase 15, "Mastery" rewards).

Commit `0a97fb9` ("fix(env): restore strict boundaries and implement
Ground Settle logic", 2026-06-14) changed `in_gate_zone` from `dist_g <
0.20` to `dist_g < 0.50`, and the `punch_holes` radius from `0.5` to `1.0`,
in the same diff that tightened `roadmark_contact` back to 0.12m. The
commit message only mentions the 0.12m restore and the Ground Settle
logic — the gate-radius widening isn't mentioned or justified anywhere,
and it moves in the opposite direction from what the message advertises
("restore strict boundaries"). Nothing in `.planning/` was updated to
reflect 0.50m/1.0m as the new intended values.

### Compounding effect

Because `punch_holes` (build-time, 1.0m) used a larger radius than
`in_gate_zone` (runtime, 0.50m even after the regression), there was a ring
between 0.20m–1.0m of every gate where:

- Real white/yellow boundary-line points were deleted from the point cloud
  entirely (`punch_holes`), so `dist_w`/`dist_y` could never read "close to
  a line" there even when the car had visibly left the road — the nearest
  remaining point in the KD-tree could be metres away.
- This has nothing to do with whether the car was actually aligned to
  cross through the gate (`alignment_ok`/`gate_hit` logic) — the point data
  was just gone, unconditionally, for any approach angle.

Net effect: a ~1–2m-diameter "no penalty, no termination" dead zone around
every intersection gate, 25x the area of the documented 0.20m design
target. This lines up with reports of the car leaving the road specifically
near intersections.

## What changed

Added a single module-level constant, `GATE_ZONE_RADIUS_M = 0.20`, in
`arcproLab/mdp/track_manager.py`, and pointed both consumers at it:

- `track_manager.py::punch_holes`: `dists > 1.0` → `dists > GATE_ZONE_RADIUS_M`
- `terminations.py::white_line_contact`: `dist_g < 0.50` → `dist_g < GATE_ZONE_RADIUS_M`
  (imported from `track_manager`)

This restores both values to the documented Phase 15 spec and removes the
possibility of the two drifting apart again, since there is now one place
to change the number.

## What was deliberately NOT changed

- `collect_raw_marker_points`'s `is_near_gate_marker` proximity check
  (1.0m, used to decide whether a *mesh* found near a gate prim should be
  bucketed as gate geometry instead of boundary geometry). This was
  introduced deliberately in `908edd0` ("Refined categorization logic with
  a 1.0m proximity threshold to prevent standard road edges from being
  misclassified as gates") as a mesh-classification tolerance, not as the
  runtime gate-crossing radius. It answers a different question ("does
  this white mesh belong to the gate at all") than `GATE_ZONE_RADIUS_M`
  ("how close must the robot be to treat a real gate as crossable"), and
  changing it wasn't part of what was asked. Flagging it here as a
  candidate for a follow-up look if boundary/gate misclassification is
  still observed after this fix.
- `RewardCfg.lateral_error`/`heading` weights (currently 0.0 in
  `arcpro_env_cfg.py`). `STATE.md` and `TODO.md` record this as an
  intentional, still-open tradeoff (waypoints are straight-line through
  turns, so those rewards falsely penalize correct turning) with its own
  planned fix (regenerate `track_centerline.npy` through intersections).
  Not touched here — separate concern, separate fix.

## Verification

Not yet run. Per Aaron's instruction, before launching anything that spins
up Isaac Sim (`ManagerBasedRLEnv`, any of the `scripts/verify_*.py` /
`scripts/train_policy.py` tools) we checked for an active training session
first:

- `ps aux` — no `train_policy.py` / Isaac Sim kit process running.
- `nvidia-smi` — no compute-app entries, GPU at 9% idle util.
- Only `arcproLab/scripts/watchdog.py` (Arika's monitor script) was running.

No training session was in progress at the time this doc was written
(2026-07-22, ~12:41). This fix has **not** been exercised in a live
rollout — that requires launching the env, which we're holding off on
until Aaron confirms it's clear to do so.

**What we could verify without touching the GPU or a live env:** importing
the real `mdp.terminations` module requires `isaaclab.envs`, which pulls in
`pxr` (USD bindings) — only importable inside a running Isaac Sim/Kit
process, so the literal shipped module can't be exercised without booting
Isaac Sim. Instead, we ran the exact masking arithmetic
(`boundary_hit & (~in_gate_zone)` with `in_gate_zone = dist_g <
GATE_ZONE_RADIUS_M`), transcribed verbatim from the post-fix file, as a
standalone CPU-only script (no Isaac Sim, no GPU context, no scene):

```
[OK] dist_g=0.15 (inside 0.20m gate zone -> stays immune): expected=False got=False
[OK] dist_g=0.30 (outside fixed 0.20m zone -> now terminates; old 0.50m bug masked this): expected=True got=True
[OK] dist_g=0.60 (well outside gate zone -> terminates): expected=True got=True
[PASS]
```

This confirms the boolean logic behaves as intended — specifically that a
boundary-line hit at 0.30m from a gate (inside the old buggy 0.50m
immunity radius but outside the correct 0.20m one) now correctly
terminates instead of being silently masked.

### Live verification (headless, after user go-ahead)

Ran `arcproLab/scripts/verify_gate_radius_fix.py` headless (`num_envs=3`,
no camera) after confirming no training was active (`ps`/`nvidia-smi`
checked immediately before launch). First run built the boundary cache
(`arcproLab/mdp/track_boundaries_1x.npz`, ~9.6MB) but its stdout never
reached the log file — Omniverse Kit's shutdown appears to hard-exit the
process before Python's fully-buffered stdout (buffered because it's
redirected to a file, not a TTY) gets flushed, so `simulation_app.close()`
can silently drop buffered `print()` output even on a clean exit. Re-ran
with `PYTHONUNBUFFERED=1` and `flush=True` on every print; second run used
the now-warm cache and produced results.

**Result: the script could not find any white boundary point within 0.50m
of a gate anywhere in the whole point cloud**, so the "near"/"mid" test
bands were empty:

```
[SETUP] near (<0.20m, should stay immune): NO POINT FOUND IN BAND
[SETUP] mid  (0.20-0.50m, SHOULD NOW TERMINATE -- this is the fix): NO POINT FOUND IN BAND
[SETUP] far  (>0.60m, control, should terminate): found @ dist_gate=105.381m
CRITICAL: could not find test points in one or more distance bands; aborting.
```

Investigated by loading the cache directly with plain `numpy` (no Isaac
Sim needed) and comparing it against the actual spawn point
(`-16.197, 5.50`, from `arcpro_env_cfg.py`) and `track_centerline.npy`:

| Query | Result |
|---|---|
| spawn -> nearest white point | 0.204m (correct — spawn straddles the lane) |
| spawn -> nearest yellow point | 0.203m (correct) |
| spawn -> nearest gate point | **117.938m** |
| centerline X/Y range (the actual driven loop) | X: [-16.20, 13.97], Y: [-18.74, 23.62] |
| all 72 discovered gate points' X range | [-193.1, -122.4] — entirely outside the driven area |

**Conclusion: `dist_g` is ~100+ everywhere on the actual driven track, in
every env. `in_gate_zone` and `gate_contact` never fire anywhere near the
robot, regardless of GATE_ZONE_RADIUS_M's value.** The gate-permeability
system this doc's fix improves is real and now matches the documented
spec, but it is currently inert on the track that's actually driven —
so it is very unlikely to be the mechanism behind the reported "car drives
off the road" symptom. The fix is still correct and worth keeping (it's a
genuine, evidenced regression against the design spec, and would matter
the moment gate discovery is fixed — see below), but it should not be
treated as *the* explanation.

### Follow-up finding: gate discovery is not scoped to env_0

The 72 gate points cluster in coordinate bands roughly 11m/19m apart
(alternating, summing to ~30) — matching `env_spacing=30.0` in
`ARCProSceneCfg(num_envs=32, env_spacing=30.0)` in `arcpro_env_cfg.py`
almost exactly. Likely cause, in `track_manager.py::collect_raw_marker_points()`:

- Gate discovery traverses `Usd.PrimRange(stage.GetPseudoRoot())` —
  the **whole stage**, all 32 replicated envs.
- White/yellow boundary-mesh collection right below it traverses
  `Usd.PrimRange(root_prim)` where `root_prim = /World/envs/env_0` —
  correctly scoped to one env.
- Both loops subtract only `env0_origin` from world positions. Gates
  picked up from env_1..env_31's replicated "laneGate" prims end up
  offset by that env's grid position minus env_0's, instead of being
  folded into a consistent per-env-local frame — which would explain both
  the ~30m-spaced clusters and why none of them land near the actual
  local track.

Not fixed in this pass — it's a separate, likely larger change (need to
decide whether to scope the gate search to `root_prim` like the boundary
search, and/or fold each env's gates into local coordinates rather than a
single shared `gate_tensor`). Flagging as the more promising lead for the
reported off-road-departure symptom, pending Aaron/Arika's direction on
priority.

### Remaining suggested verification once this deeper issue is addressed

1. `arcproLab/scripts/verify_gates.py` (GUI) to visually confirm gate
   marker positions land on the actual driven track once discovery is
   scoped correctly.
2. Re-run `arcproLab/scripts/verify_gate_radius_fix.py` — the near/mid
   bands should populate once real gates exist near the track, and should
   pass with the 0.20m radius fixed in this doc.
3. Re-run `arcproLab/scripts/test_intersection_crossing.py` for gate
   pass-through vs. wall-crash regression coverage.
