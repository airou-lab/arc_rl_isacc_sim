# Phase 2 (IN PROGRESS): centerline regeneration — investigation findings

**Branch:** `off-road-debug`
**Status:** Investigation complete, implementation **not** done. No repo files
changed by this doc. Design decision pending.

Phase 2 turned out to be materially harder than Phases 0/1. This records what
was learned so the work isn't repeated.

## Why the current file is broken (root cause, not tuning)

Both existing generators (`fix_centerline.py`, `arcproLab/scripts/generate_centerline.py`)
build the path the same way:

1. for each **yellow** point, find the **nearest white** point,
2. take the **midpoint** as a lane-center candidate,
3. **greedily chain** nearest-neighbours from a start point,
4. compute yaw via `arctan2` between *consecutive* candidates.

Every one of those four steps is a failure source, and the damage is
measurable in the shipped file: 178 fragments, median fragment 2 waypoints,
29.3% of curvature values physically impossible.

**Nearest-neighbour paint pairing is not reliable.** In dense-marking regions
(intersections, dashed lines, turn arrows) a yellow point's nearest white
neighbour is frequently *not* the opposite boundary of its own lane. That
produces candidates that are not lane centers at all. This is a structural
flaw in the method, not a parameter that needs tuning — which is why the
existing file is fragmented rather than merely noisy.

Confirmed empirically: a reimplementation with heading-continuity chaining
(much stricter than the existing greedy version) still drifted off the true
lane center at X=-16.19 onto candidates at **X=-15.99 — the yellow line
itself** — and stalled after only 11.6 m of the ~1076 m file.

## Ground truth measured from the boundary cloud

Cross-section at the spawn (Y=5.50), from `track_boundaries_1x.npz`:

```
white(-16.41)  |  yellow(-15.97)  |  white(-15.53)
```

- Two-lane road, lane width **~0.44 m**.
- Spawn X = **-16.197** is exactly the midpoint of the **right lane** for a
  car heading South (-Y). The spawn is correct.

## Two data hazards found

1. **22.7% of "white" points are not road paint.** 56,237 of 248,127 white
   points sit at Z > 0.1 m, up to **Z = 2.5 m** — vertical structures pulled
   in by the `"roadmarks"/"paint"/"marking"` path-name filter in
   `collect_raw_marker_points`. They do **not** cause false terminations
   (`compute_marker_distances` uses 3-D distance, so a 2.5 m-high point is
   far away in Z), but they corrupt any 2-D pairing. **Any generator must
   filter to ground level (Z < 0.1).** Neither existing generator does.
2. **Intersections have no yellow centerline**, so the midpoint method has
   *no candidates at all* there — a genuine data gap, not a chaining bug.
   Bridging intersections is unavoidable, and it is precisely what the open
   `TODO.md` item asks for ("map the right-turn intersection into
   track_centerline.npy so the reward function correctly rewards turning").

## No authoritative source to fall back on

The gate prims carry `ODMapLaneID` / `signal_stop_point_laneID` attributes,
which suggested the USD might contain OpenDRIVE lane geometry worth using
instead of reconstructing from paint. Checked the flattened stage: it
contains only the 24 `laneGate_*` Xforms and no centerline, reference-line,
or lane-geometry curves. **Reconstruction from paint is the only option.**

## Proposed approach (not yet implemented)

Replace midpoint-pairing with **single-line tracing plus lateral offset**:

1. Filter yellow and white to ground level (Z < 0.1).
2. Trace **one** line — the yellow centre divider — into an ordered polyline.
   Ordering a single continuous curve is far more robust than pairing two
   noisy clouds, and removes the cross-road jump failure mode entirely.
3. Smooth, then resample to uniform ~0.10 m spacing (vs today's ragged 1 cm,
   which is what amplifies yaw quantization noise ~100x into kappa).
4. Compute yaw by **central difference over a window** (~±0.3 m baseline)
   rather than between adjacent points.
5. Offset laterally by **half a lane width (~0.22 m)** to the right-lane side
   to produce the drivable reference path.
6. Bridge intersection gaps by heading-consistent interpolation.

**Validation plan (all offline, no Isaac Sim):** the produced path must be
~0.22 m from the nearest yellow *and* ~0.22 m from the nearest outer white
along its length; contain 0 fragments; have uniform spacing; and yield a
curvature distribution that is ~100% physically plausible (vs today's 70.7%),
making the Phase 1 kappa clamp inert rather than load-bearing.

## Open decision

This is a meaningful piece of work (a new generator plus validation), and it
rewrites the reference every downstream reward and observation depends on.
Flagging for Aaron/Arika before proceeding, rather than committing to an
approach unilaterally.
