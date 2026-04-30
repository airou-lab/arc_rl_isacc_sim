# Codebase Concerns

**Analysis Date:** 2026-04-29

## Tech Debt

**USD Scaling Hack (Tracked: 14-01):**
- Issue: The track USD is scaled by `0.125` in the environment config to match the 1.0x metric robot. This implies the original USD was authored at 8x scale.
- Files: `arcproLab/arcpro_env_cfg.py`, `openStreetUSD/no_graph_sim_clean_1x.usda`
- Impact: Confusion regarding "true" metric units. If the USD is ever re-exported at 1.0x, the environment will break.
- Fix approach: Flatten the track USD to natively be 1.0x metric scale and remove the config-level scaling `scale=(0.125, 0.125, 0.125)`.
- Reference: `.planning/todos/14-01-RESEARCH.md`

**Telemetry Vector Placeholders (Tracked: 14-03):**
- Issue: Index 10 (Curvature/Kappa) is hardcoded to 0.0.
- Files: `arcproLab/mdp/observations.py`, `arcproLab/mdp/track_manager.py`
- Impact: Policy lacks awareness of upcoming track geometry, limiting proactive steering on turns.
- Fix approach: Implement curvature calculation (`kappa = d_theta / d_s`) in `TrackManager` using a ~2.0m lookahead over track waypoints.
- Reference: `.planning/todos/14-03-RESEARCH.md`

**RoadGraph Placeholder:**
- Issue: `RoadGraph` currently returns a constant "Straight" intent.
- Files: `arcproLab/mdp/road_graph.py`
- Impact: Robot cannot navigate intersections or follow missions.
- Fix approach: Implement the trigger-based decision logic as planned in Phase 11.

## Known Bugs

**Inverted Speeds (Potential):**
- Issue: Debug logic in `observations.py` checks for "action/speed inversion".
- Symptoms: Robot might accelerate backwards when given forward throttle.
- Files: `arcproLab/mdp/observations.py`
- Trigger: Incorrect actuator joint expression or motor direction in USD.
- Workaround: Monitored via telemetry debug prints.

## Security Considerations

**Simulation Escape:**
- Risk: Robot falling into the void if `ground_plane` is removed (which it was in latest config).
- Files: `arcproLab/arcpro_env_cfg.py`
- Current mitigation: Robust termination logic in `terminations.py` to reset on fall.
- Recommendations: Keep a visual ground plane at low Z for visual reference even if collisions are disabled.

## Performance Bottlenecks

**TrackManager Boundary Collection (Tracked: 14-02):**
- Problem: Collecting 10k+ points from USD stage on startup causes a 5-10 second blocking delay.
- Files: `arcproLab/mdp/track_manager.py`
- Cause: Iterating through every mesh vertex in the track USD.
- Improvement path: Cache the extracted boundaries and gates to `.npy` files instead of re-calculating on every run, using a `--rebuild_track_cache` mechanism.
- Reference: `.planning/todos/14-02-RESEARCH.md`

**Marker Visualization:**
- Problem: Drawing 1000s of spheres in the GUI.
- Files: `arcproLab/mdp/track_manager.py`
- Cause: Individual sphere markers for lane lines.
- Improvement path: Only enabled via `--debug` flag; use `PointInstancer` or `VisualizationMarkers` more efficiently.

## Fragile Areas

**Implicit Actuators:**
- Files: `arcproLab/arcpro_robot_cfg.py`
- Why fragile: Gains (stiffness/damping) are extremely sensitive to scaling. 1.0x metric scale requires much lower gains than legacy 8x or 100x scales.
- Safe modification: Use `verify_drive.py` and `audit_wheels.py` after any change to actuator configs.
- Test coverage: Low (manual verification).

## Scaling Limits

**Metric Scale:**
- Current capacity: 1.0x metric.
- Limit: Floating point precision in PhysX for very small objects (e.g., 0.01m).
- Scaling path: Maintain 1.0x but use smaller simulation timesteps if instability occurs.

## Missing Critical Features

**Intersection Logic:**
- Problem: No mechanism to select lanes at a junction.
- Blocks: Autonomous navigation through the full `no_graph_sim` environment.

**Dynamic Obstacles:**
- Problem: Env only supports a single ego-robot.
- Blocks: Multi-agent training.

## Test Coverage Gaps

**Actuator Physics:**
- What's not tested: Real-world torque/velocity matching.
- Files: `arcproLab/arcpro_robot_cfg.py`
- Risk: Policy learns "sim-only" physics that doesn't transfer to hardware (Reality Gap).
- Priority: High.

---

*Concerns audit: 2026-04-29*