# Codebase Concerns

**Analysis Date:** 2025-05-20

## Redundant and Temporary Files

**`trash/` Directory:**
- Issue: Contains 100+ legacy scripts and old USD files from previous phases (e.g., 8x scale logic, old physics fixes).
- Files: `trash/` (recursive)
- Impact: Clutters the repository and confuses developers looking for active tools.
- Fix approach: Securely archive or remove the directory before Phase 11.

**Duplicate Cleanup Scripts:**
- Issue: Multiple versions of terrain/environment cleaning scripts exist.
- Files: `arcproLab/scripts/clean_terrain.py` vs `arcproLab/scripts/clean_terrain_v2.py`.
- Impact: Inconsistency in environment preparation.
- Fix approach: Retain `v2` and remove the original.

**Duplicate Asset Audits:**
- Issue: Overlap between scripts that audit USD assets.
- Files: `arcproLab/scripts/audit_assets.py` vs `arcproLab/scripts/audit_live.py`.
- Impact: Redundant functionality.
- Fix approach: Merge into a single `audit_usd.py` or similar.

**Scratch/Working Files:**
- Issue: Temporary files from testing remain in the root or `arcproLab/`.
- Files: `arc_rl_isacc_sim/tm_working.py`, `arc_rl_isacc_sim/test_sphere.py`, `arcproLab/scripts/agent_view.png`.
- Impact: Repository "hygiene" issues.
- Fix approach: Remove after verification or move to a dedicated `scratch/` folder ignored by git.

## Duplicate Logic (mdp vs scripts)

**Yaw Calculation:**
- Issue: The math to convert quaternions to yaw is repeated in at least 5 locations.
- Files: `arcproLab/mdp/observations.py`, `arcproLab/mdp/terminations.py`, `arcproLab/scripts/diagnose_lane.py`, `arcproLab/scripts/verify_spawn.py`.
- Impact: If the Z-up convention or rotation logic changes, it must be fixed in all places.
- Fix approach: Move to a central utility in `arcproLab/mdp/utils.py` or similar.

**Lateral Error Calculation:**
- Issue: `TrackManager.compute_errors` is called independently by observations and terminations.
- Files: `arcproLab/mdp/observations.py`, `arcproLab/mdp/terminations.py`.
- Impact: Redundant computation.
- Fix approach: Terminations should read `env.extras["lat_err"]` which is already populated by observations.

**USD Stage Traversal:**
- Issue: Multiple scripts traverse the stage looking for "yellow" or "white" markers.
- Files: `arcproLab/mdp/track_manager.py`, `arcproLab/scripts/check_camera_and_lanes.py`.
- Impact: Slow initialization and duplicate search logic.
- Fix approach: Use `TrackManager` as the single source of truth for track geometry markers.

## Technical Debt

**Hardcoded Waypoint Paths:**
- Issue: `TrackManager` has a hardcoded path to `track_centerline_1x.npy`.
- Files: `arcproLab/mdp/track_manager.py`
- Impact: Fragile if file structure changes.
- Fix approach: Pass path via `ARCProEnvCfg`.

**TrackManager Auto-Centering:**
- Issue: The `generate_centerline` logic is complex and runs during environment initialization.
- Files: `arcproLab/mdp/track_manager.py`
- Impact: Can slow down startup; failure to find markers leads to fallback waypoints which might be misaligned.
- Fix approach: Move centerline generation to a pre-processing script that saves a validated `.npy` file.

## Logic Bugs

**Masked Observations in Debug Tools:**
- Issue: `debug_terminations.py` reads `obs[:, 8]` to check for marker hits, but `observations.py` masks this index to 0.0 to force vision-only driving.
- Files: `arcproLab/mdp/debug_terminations.py`, `arcproLab/mdp/observations.py`.
- Impact: `debug_termination` will never detect a marker hit.
- Fix approach: `debug_termination` must use `env.extras["lat_err"]`.

## Scaling Gaps (Phase 11 Preparedness)

**Intersection Support:**
- Issue: `TrackManager` assumes a single sequence of waypoints.
- Files: `arcproLab/mdp/track_manager.py`
- Impact: Will fail at intersections where multiple paths exist.
- Fix approach: Upgrade `TrackManager` to support branching graphs or dynamic target selection for Phase 11.

---

*Concerns audit: 2025-05-20*
