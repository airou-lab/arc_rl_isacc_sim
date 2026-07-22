# Fix: gate discovery scoped to whole stage instead of env_0

**Branch:** `off-road-debug`
**Files touched:** `arcproLab/mdp/track_manager.py`
**Status:** Implemented and validated single-env. Does NOT explain the
reported off-road-departure symptom — see Bottom line below.

## Problem this addresses

Follow-up from `01-gate-radius-hole-punching.md`. Live verification there
showed `dist_g` (distance to nearest gate) is ~118m near the robot's spawn
point, meaning gate-permeability logic never engages anywhere on the
driven track. That doc's live test ran with `num_envs=3`, and the 72
discovered gate points formed exactly 3 clusters of 24, spaced ~30m apart
— matching `env_spacing=30.0` in `arcpro_env_cfg.py`'s
`ARCProSceneCfg(num_envs=32, env_spacing=30.0)` almost exactly. That
pattern pointed at a scoping bug.

## Root cause

In `track_manager.py::collect_raw_marker_points()`, the gate-prim search
loop traversed the *entire* USD stage:

```python
# 1. First, find all Gates explicitly (Global Search like verify_gates.py)
...
for prim in Usd.PrimRange(stage.GetPseudoRoot()):
    ...
```

while the white/yellow boundary-mesh collection loop right below it
correctly scopes to one env:

```python
for prim in Usd.PrimRange(root_prim):   # root_prim = /World/envs/env_0
```

Both loops subtract only `env0_origin` from every world position they
find. For the boundary loop this is correct, because it only ever visits
`env_0`'s own geometry. For the gate loop, with `num_envs > 1`, this picks
up "laneGate" prims replicated under every other instanced env
(`env_1`...`env_N-1`) too, each only having `env_0`'s origin subtracted —
so their reported positions land offset by that env's grid position minus
`env_0`'s, rather than folding back into a shared local frame.

The comment ("Global Search like verify_gates.py") explains how this
happened: `arcproLab/scripts/verify_gates.py` is a GUI tool that
visualizes every gate across the whole stage — a whole-stage search is
correct for that tool's purpose. `collect_raw_marker_points` copied that
pattern (introduced in the same commit, `908edd0`, that built the whole
gate-permeability feature) without adjusting it for the fact that its
output needs to be a single canonical *local-frame* template, like the
boundary-mesh loop already produces — not a global raw-world-position
dump.

## What changed

`track_manager.py::collect_raw_marker_points`: gate-prim search loop
changed from `Usd.PrimRange(stage.GetPseudoRoot())` to
`Usd.PrimRange(root_prim)`, matching the boundary-mesh loop's scoping
exactly. Comment updated to explain why and point back here.

## Verification

Per Aaron: validate single-agent/single-env behavior first, since that's
easier to reason about and is a prerequisite for trusting any multi-env
result. Checked `ps`/`nvidia-smi` immediately before each launch (nothing
running throughout). Deleted the stale `track_boundaries_1x.npz` cache
(gitignored, built during the previous doc's `num_envs=3` test, so it
still had 3 envs' worth of contaminated gates baked in) so the next run
rebuilds fresh under the fix.

Ran `arcproLab/scripts/verify_gate_discovery_scoping.py`, headless,
`num_envs=1` (no other env exists to contaminate from even before this
fix, so this run isolates whether real, local gates exist near the track
at all once the search is properly scoped):

```
[TrackManager] Found 24 gate prims. Example: /World/envs/env_0/Track/sub_intersections/network/junction_18/laneGate_23
[TrackManager] Finalized Yellow: 151764 points.
[TrackManager] Finalized White: 248127 points.
[TrackManager] No gate meshes found. Using prim centers as fallback.
[RESULT] white points: 248127
[RESULT] yellow points: 151764
[RESULT] gate points: 24
[RESULT] spawn [-16.18, 5.30] -> nearest gate: 118.041m
[RESULT] centerline-to-nearest-gate: min=112.454m max=147.074m mean=129.600m
[RESULT] centerline waypoints within GATE_ZONE_RADIUS_M (0.2m) of a gate: 0 / 95045
```

The fix is confirmed working as intended: gate prim paths now correctly
read `/World/envs/env_0/Track/...` (env-scoped, as opposed to the earlier
72-point/3-cluster contamination), and the count (24) is a clean, sensible
number for one env instead of a multiple of it.

## Bottom line: this isn't the road-departure bug either

Even fully fixed and scoped to a single env, **every one of the 24 tagged
gates is 112m+ away from every single one of the 95,045
`track_centerline.npy` waypoints** — i.e., the entire closed loop the
robot actually drives never comes anywhere near a tagged intersection.
Zero waypoints are within `GATE_ZONE_RADIUS_M` of any gate.

This means gate-permeability logic (`in_gate_zone`, `gate_contact`,
`gate_hit`, and by extension the `01-gate-radius-hole-punching.md` radius
fix) is structurally inert for the track currently being trained on,
independent of `num_envs`, independent of both fixes in this branch so
far. The 24 `DSLaneGate`-tagged junctions appear to be leftover from
earlier Phase 11 intersection work on a different/larger section of the
source city map; `STATE.md` describes the current active phase as "Pure
Vision-Based Lane Following," consistent with the currently-driven loop
being a plain lap with no live intersection in it.

**Consequence for the original hypothesis:** the "agent conflates white
road boundaries with white stop-line gates" theory Aaron and Arika
discussed cannot be the operative mechanism right now — there's no gate
near the driven loop for anything to be confused with. Both fixes in this
branch are real, evidenced, worth keeping (they're correct against the
Phase 15 design spec and against the boundary-mesh loop's own scoping
pattern respectively), but neither explains the reported symptom. Next
investigation should redirect toward:

1. The reward-shaping side (`lateral_error`/`heading` weights zeroed,
   `TODO.md`'s open item about `track_centerline.npy` not correctly
   representing the turn), or
2. The action/joint-mapping and physics side (`TODO.md`'s open item on
   "slight left-drift bias... driving perfectly straight with 0.0
   steering" — closer to the "expected vs. real value" comparison Aaron
   originally proposed).
