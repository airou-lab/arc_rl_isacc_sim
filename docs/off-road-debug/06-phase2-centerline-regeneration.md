# Phase 2: centerline regeneration

**Branch:** `off-road-debug`
**Added:** `arcproLab/scripts/generate_centerline_v2.py`
**Regenerated:** `arcproLab/mdp/track_centerline.npy` (old copy kept at
`track_centerline.npy.pre_v2.bak`)
**Status:** Implemented and validated, offline **and** live. **Not committed.**

Root-cause fix for the fragmented track reference diagnosed in
`05-phase2-centerline-investigation.md`.

## Scope decision: single loop, and why it is not a compromise

Supporting the **full road network** is not a generator-scope choice — it is a
**consumer-architecture** change. `TrackManager.compute_errors` tracks the
nearest waypoint with a windowed search (`last_indices` ±50) over a single
ordered array, so index-adjacency must equal spatial-adjacency along the
driven route. A branching network cannot be represented in that structure;
supporting it would require redesigning the windowed tracker, `lat_err`
semantics, and every reward that consumes them.

So v2 emits **one closed route**. The tracer internals are route-agnostic, so
a later multi-route version can reuse them without a rewrite.

## Method: virtual lane-follower (not candidate chaining)

Candidate chaining — what both old generators do — was shown in
`05-...md` to be structurally unsound. A stricter reimplementation with
heading-continuity still drifted onto the yellow line and stalled at 11.6 m.

v2 instead **simulates a car tracking a lane**: walk forward in 5 cm steps and
servo the lateral offset against the yellow centre divider. Curves are
followed naturally, and short paint gaps (intersections) are coasted through
on heading. It closed a loop on the first attempt.

Pipeline: `trace -> uniform arclength resample -> periodic smooth ->
yaw by central difference over a window -> curvature`.

The yaw window is the key noise fix: deriving yaw over a **0.7 m baseline**
instead of between adjacent ~1 cm, 1 mm-quantized points removes the ~100x
noise amplification that `kappa = dyaw/ds` was applying.

Runs **fully offline** (pure numpy/scipy against the cached boundary cloud),
so iterating on the track reference needs no Isaac Sim boot.

## Tuning: chosen by sweep, not by eye

Smoothing trades curvature quality against corner-cutting. Sweeping both
windows (`|k|max` / % plausible / median distance to yellow & white):

| smooth_w | yaw_w | \|k\|max | plausible | yellow | white |
|---|---|---|---|---|---|
| 5 | 5 | 2.00 | 98.3% | 0.208 | 0.199 |
| **5** | **7** | **1.49** | **100.0%** | **0.208** | **0.199** |
| 9 | 7 | 1.45 | 100.0% | 0.208 | 0.199 |
| 13 | 7 | 1.40 | 100.0% | 0.208 | 0.198 |
| 17 | 7 | 1.18 | 100.0% | 0.211 | 0.196 |
| 21 | 7 | 0.97 | 100.0% | **0.220** | **0.186** |

Heavier smoothing keeps improving curvature but starts **pulling the path off
centre** (0.208/0.199 -> 0.220/0.186 at window 21) — corner-cutting. Chose the
**lightest** setting that reaches 100% plausible, preserving centring exactly
and shrinking the loop only 0.16 m.

## Results

| Metric | Old | New |
|---|---|---|
| Waypoints | 95,045 (2.28 MB) | **480** (11.6 KB) |
| Loop length | 1076.66 m (wandering) | 47.98 m (closed circuit) |
| Fragments | **178** (median 2 waypoints) | **0** |
| Spacing | 1 cm median, ragged to 1.31 m | uniform 0.10 m |
| Loop closure | 16.2 m gap (while curvature wrapped `% n`) | **closed** |
| Physically plausible curvature | **70.7%** | **100.0%** |
| \|kappa\| max | 1577 (0.6 mm radius) | **1.49** (0.67 m radius) |
| Distance to yellow / white | n/a | 0.208 / 0.199 (p95 == median) |

`p95 == median` on both boundary distances means the path has essentially
**zero wander** from lane centre around the entire loop.

Loop closure also incidentally fixes a latent bug: `ensure_synced` computes
curvature with `i1 = (i + 1) % n_wps`, which assumes periodicity. The old
file had a 16.2 m start/end gap, so that wraparound produced one garbage
value. A genuinely closed loop makes the existing code correct.

## Live validation

Re-ran `probe_baseline.py` (headless, `num_envs=1`). The decisive line:

```
[TrackManager] Calculated curvature for 480 waypoints (0 clamped to +/-2.0).
```

**0 clamped, down from 27,804 (29.3%).** The Phase 1 kappa guard is now
completely **inert** — the correct outcome when a root cause is genuinely
fixed rather than masked. The guard stays in as a regression tripwire.

Slot 10 also came alive. It previously read a flat `0.000` and was flagged
`CONSTANT (dead slot)`; it now carries real curvature:

```
10 kappa   min=-0.0016  max=0.0313  range=0.0329
```

All Phase 1 assertions still pass (slot 11 ratio 1.002, all slots finite),
confirming no regression.

## Safety

- The generator **validates before writing** and refuses on any failed check,
  including refusing to write a non-closed loop.
- Backed up the previous file to `track_centerline.npy.pre_v2.bak`.
- The old file is git-tracked and therefore doubly recoverable. Note it is
  covered by the `*.npy` ignore rule, so committing the new one needs
  `git add -f`.

## Consequence for Phase 3

`lat_err`, `head_err` and `kappa` now derive from a trustworthy, continuous,
physically-drivable reference. That was the blocker on re-enabling the
centring rewards (`lateral_error` / `heading`, both currently weight 0.0) —
the original reason for zeroing them was that straight-line waypoints
penalised correct turning. A 100%-plausible closed loop removes that
objection, so Phase 3 can proceed.
