# Phase 1: observation integrity fixes

**Branch:** `off-road-debug` (on top of 115c49a)
**Files touched:** `arcproLab/mdp/observations.py`, `arcproLab/mdp/track_manager.py`,
`arcproLab/scripts/probe_baseline.py` (validation only)
**Status:** Implemented and validated. **Not committed** — pending review.

Scope was deliberately limited to observation integrity. The drift-bias fix
was held back so its effect stays independently measurable; mixing a
physics/asset change into this batch would confound both results.

## Fix 1 — slot 11 `distance`: sign + timestep

`observations.py` accumulated distance as:

```python
env.extras["distance"] += asset.data.root_lin_vel_b[:, 0] * 0.05
```

Two defects, both measured in `03-phase0-baseline.md`:

1. **Sign inverted.** Forward is local **−X** (the convention `speed_reward`
   already uses via `forward_speed = -local_vel[:, 0]`), so this accumulated
   *negative* distance while driving forward. The baseline probe measured
   slot 11 running −1.62 → −15.26 across 5.44 m of forward travel.
2. **Wrong timestep.** The hardcoded `0.05` is a 20 Hz step, but the control
   rate is `sim.dt * decimation = 0.002 * 10 = 0.02` s (50 Hz) — inflating
   distance by 2.5×. The baseline measured a ratio of **2.505** against
   ground truth.

Now:

```python
env.extras["distance"] += -asset.data.root_lin_vel_b[:, 0] * env.step_dt
```

`env.step_dt` is used rather than a hardcoded `0.02` deliberately: hardcoding
is how the original bug survived a decimation change, so reading the value
from the environment prevents the same class of drift recurring.

## Fix 2 — slot 10 `kappa`: physical-range guard

Added `MAX_PATH_CURVATURE = 2.0` in `track_manager.py` and clamped curvature
**at the source** (where `curvature_tensor` is built) so every consumer
benefits, not just the observation.

Bound is derived, not guessed: the vehicle's minimum turn radius is
`wheelbase / tan(max_steering_angle) = 0.33 / tan(0.5) = 0.604 m`, i.e.
`|kappa| <= 1.66`. No drivable centerline should demand a tighter radius
than the car can physically execute, so 2.0 (radius >= 0.5 m) leaves
headroom while rejecting noise spikes.

**This is a guard, not a fix.** The diagnostic it prints quantifies how bad
the underlying data is:

```
[TrackManager] Calculated curvature for 95045 waypoints (27804 clamped to +/-2.0).
```

**29.3% of all waypoints demand a physically impossible turn** — matching
the independently-derived 70.7%-plausible figure exactly. The real remedy
is regenerating `track_centerline.npy` (Phase 2).

## Validation

Re-ran `probe_baseline.py` (headless, `num_envs=1`, 400-step forward
rollout). Added Part E: falsifiable assertions rather than eyeballed output.

```
[PASS] slot11 sign: min=0.649 max=6.102 (must rise and end positive while driving forward)
[PASS] slot11 scale: accumulated=5.453 true_displacement=5.443 ratio=1.002 (want ~1.0, was 2.505 pre-fix)
[PASS] slot10 kappa within +/-2.0: min=0.000 max=0.000
[PASS] all slots finite: 0 non-finite readings

[PASS] Phase 1 observation-integrity checks all passed.
```

Slot 11 before vs after, same probe, same conditions:

| | min | max | vs ground truth |
|---|---|---|---|
| Before | −15.256 | −1.623 | ratio **2.505**, sign inverted |
| After | +0.649 | +6.102 | ratio **1.002**, sign correct |

Accumulated 5.453 against a true displacement of 5.443 m.

One rigor guard added to the check itself: `distance` zeroes on episode
reset, so a run containing a reset would make the scale ratio meaningless.
That assertion now skips itself and says so rather than reporting a bogus
pass. (This run had zero resets, so it applied.)

## Unchanged, deliberately

- **Slots 1 (`go_signal`) and 2 (`goal_dist`)** remain hardcoded constants.
  The 2026-07-01 session log records these as reserved placeholders for V2I
  work (slot 2 "reserved for ttc/cycle_s"), so they are intentional, not
  defects. Changing them would renegotiate the 12-dim observation contract.
- **Slots 8/9 (`lat_err`, `head_err`)** remain masked — the deliberate
  vision-only mandate per `STATE.md`. These are Phase 3 candidates once the
  centerline is trustworthy.
- **Slot 11 remains unbounded/monotonic**, growing to ~600 over a full
  15,000-step episode. A non-stationary feature is poor practice under
  `VecNormalize`, but bounding it changes the observation's meaning, which
  is a design decision for Aaron/Arika rather than a correctness fix.
  Flagged, not changed.

## New evidence: slot 0 `turn_token`

Still **not proven** to re-randomize per episode — both runs had zero
resets. But it read `0.0` in the Phase 0 run and `1.0` in this run, under
identical conditions. That cross-run variation is consistent with
`RoadManager.__init__` randomizing at construction, and supports (without
proving) the concern that it injects an unactionable random constant while
gates are inert. Confirming it needs a probe that forces episode resets.

## Status of the issue list

| Issue | Status |
|---|---|
| Slot 11 sign + 2.5× scale | **FIXED & VALIDATED** (ratio 1.002) |
| Slot 10 kappa spikes | **GUARDED** (29.3% clamped) — root cause open for Phase 2 |
| Steering -> joint mapping | Ruled out (Phase 0) |
| Centerline fragmentation | **OPEN — Phase 2, the root cause** |
| Left-drift bias | Confirmed/quantified, **deliberately not yet fixed** |
| Reward gap (no centering incentive) | **OPEN — Phase 3**, leading suspect for the symptom |
| Slot 0 turn_token | Suggestive cross-run evidence, still unproven |
