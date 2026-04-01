# Codebase Concerns

**Analysis Date:** 2024-10-24

## Tech Debt

**[Scale Discrepancy]:**
- Issue: Documentation reflects **0.5x robot scale** and **Front-Wheel Drive (FWD)**, but `arcpro_env_cfg.py` and `arcpro_robot_cfg.py` may still contain `scale=(1.0, 1.0, 1.0)` and `Joint_Drive_.*` (4WD) references.
- Files: `arcproLab/arcpro_env_cfg.py`, `arcproLab/arcpro_robot_cfg.py`.
- Impact: Potential confusion during implementation or if assets are swapped without updating the config classes.
- Fix approach: Update config classes to explicitly use 0.5x scale and restrict throttle to `Joint_Drive_FL/FR`.

**[Track Origin Shift]:**
- Issue: `TrackManager` currently contains logic to shift waypoints to the origin.
- Files: `arcproLab/mdp/track_manager.py`.
- Impact: Incompatible with **Absolute Waypoint Alignment** required for multi-map/large-scale simulation.
- Fix approach: Remove the coordinate shift in `sample_waypoints_from_usd()` and ensure `track_centerline.npy` contains absolute world coordinates.

## Known Bugs

**[OSM Invisible Barriers]:**
- Issue: Tile junctions in OSM-generated tracks sometimes have "invisible barriers" or collisions.
- Files: `openStreetUSD/` assets.
- Trigger: Robot crossing a tile boundary.
- Workaround: Using `no_graph_sim_final.usd` which is flattened and cleaned.

## Security Considerations

**[Local Simulation]:**
- Risk: None identified; simulation runs locally with standard dependencies.

## Performance Bottlenecks

**[Solvers]:**
- Problem: High precision solvers (32 pos, 16 vel iterations) are enabled for the robot.
- Files: `arcproLab/arcpro_robot_cfg.py`.
- Cause: Required for stable physics at 0.5x scale.
- Improvement path: Optimize collision geometry (using primitive hulls) to reduce simulation load.

## Fragile Areas

**[Waypoint Precision]:**
- Files: `arcproLab/mdp/track_centerline.npy`.
- Why fragile: If the track USD is updated but the centerline is not re-generated, navigation will fail.
- Test coverage: `tests/test_track_manager.py` checks logic but not data validity.

## Scaling Limits

**[Hierarchical Bottleneck]:**
- Current capacity: Efficient path planning at 0.5m spacing.
- Limit: Very high-speed cornering where waypoint density may be insufficient.
- Scaling path: Dynamically adjust `num_waypoints` or `waypoint_horizon` based on vehicle speed.

## Missing Critical Features

**[Lane-Aligned Spawning]:**
- Problem: Jitter spawning with lateral and heading noise relative to waypoints is not yet implemented.
- Files: `arcproLab/mdp/events.py`.
- Blocks: Domain randomization for more robust policies.

## Test Coverage Gaps

**[12-Float Protocol]:**
- What's not tested: The observation mapping is complex and prone to index-off-by-one errors.
- Files: `arcproLab/mdp/observations.py`.
- Risk: Incorrect telemetry leads to policy instability.
- Priority: High.

---

*Concerns audit: 2024-10-24*
