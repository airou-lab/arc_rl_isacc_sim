# Codebase Concerns

**Analysis Date:** 2025-05-21

## Redundant and Temporary Files

**`trash/` Directory:**
- Issue: Contains 100+ legacy scripts and old USD files from previous phases.
- Files: `trash/` (recursive)
- Impact: Clutters the repository and obscures active tools.
- Fix approach: Archive or remove the directory.

**Duplicate Cleanup Scripts:**
- Issue: Multiple versions of terrain/environment cleaning scripts exist.
- Files: `arcproLab/scripts/clean_terrain.py` vs `arcproLab/scripts/clean_terrain_v2.py`.
- Impact: Inconsistency in environment preparation.
- Fix approach: Retain `v2` and remove the original.

## Logic Bugs and Technical Debt

**Broken Unit Tests:**
- Issue: `tests/test_track_manager.py` still expects centerline-based `compute_errors` and `get_closest_waypoint_data`, which are now stubbed/deprecated in favor of marker-based proximity.
- Files: `tests/test_track_manager.py`, `arcproLab/mdp/track_manager.py`.
- Impact: Unit tests give false positives or fail, making them useless for CI.
- Fix approach: Refactor tests to validate `compute_marker_distances` and `collect_raw_marker_points`.

**Repeated Yaw Calculation:**
- Issue: Quaternion-to-yaw math is repeated in multiple locations.
- Files: `arcproLab/mdp/observations.py`, `arcproLab/mdp/terminations.py`, `arcproLab/scripts/diagnose_lane.py`.
- Impact: Maintenance burden; risk of divergence in coordinate frame logic.
- Fix approach: Move to a central utility in `arcproLab/mdp/utils.py`.

**Legacy Centerline Stubs:**
- Issue: `TrackManager.compute_errors` and `observations.py` still contain "lat_err" and "head_err" logic (even if masked).
- Files: `arcproLab/mdp/track_manager.py`, `arcproLab/mdp/observations.py`.
- Impact: Confusing for new contributors; code paths exist that are no longer functional.
- Fix approach: Completely remove `compute_errors` and refactor the telemetry vector to either remove or formally redefine indices 8 & 9.

## Scaling Gaps (Phase 11 Preparedness)

**Intersection Support:**
- Issue: `TrackManager` assumes a global set of markers but doesn't handle branching logic for intersections.
- Files: `arcproLab/mdp/track_manager.py`
- Impact: In Phase 11, the robot might be "too close" to a marker of a crossing lane, triggering false terminations.
- Fix approach: Implement marker segmentation or path-specific boundary sets.

**Hardcoded Fallback Waypoints:**
- Issue: `TrackManager` has a single hardcoded fallback waypoint: `torch.tensor([[-16.25, 5.56, -1.57]])`.
- Files: `arcproLab/mdp/track_manager.py`
- Impact: Environment initialization will be "wrong" if USD markers are not found.
- Fix approach: Enforce a strict error if markers cannot be collected or use a dynamic spawn-point discovery.

---

*Concerns audit: 2025-05-21*
