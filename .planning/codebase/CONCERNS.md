# Codebase Concerns

**Analysis Date:** 2025-04-05

## Tech Debt

**[Scale Discrepancy (Visual vs Physics)]:**
- Issue: While physics and assets are configured for **1.0x metric scale**, `arcpro_env_cfg.py` uses an **8.0x scale override** for the robot to match the visual scale of the current track USD.
- Files: `arcproLab/arcpro_env_cfg.py`.
- Impact: Inconsistency between "True Metric" intent and actual simulation parameters. Telemetry might require scaling factors if world units don't match meters.
- Fix approach: Re-export track USD at true 1.0x metric scale and revert robot scale to 1.0x in environment config.

**[Actuator Effort Limits]:**
- Issue: `effort_limit_sim` for 4WD throttle is set to 2000.0, which might be extremely high even for a 20kg robot.
- Files: `arcproLab/arcpro_robot_cfg.py`.
- Impact: Over-powered motors can lead to unrealistic acceleration or physics instability if not properly tuned.
- Fix approach: Calibrate effort limits based on real F1Tenth motor specifications adjusted for the 20kg mass.

## Known Bugs

**[OSM Invisible Barriers]:**
- Issue: Tile junctions in OSM-generated tracks sometimes have "invisible barriers" or collisions.
- Files: `openStreetUSD/` assets.
- Trigger: Robot crossing a tile boundary.
- Workaround: Phase 07 "Hardening" process applied via `repair_usd_references.py`.
- Status: Partially mitigated, still present in some legacy OSM assets.

**[Dead F1Tenth Folder Reference]:**
- Status: **Resolved in Phase 07** via `repair_usd_references.py` which removes the missing `/World/F1Tenth` prim.

## Security Considerations

**[Local Simulation]:**
- Risk: None identified; simulation runs locally with standard dependencies.

## Performance Bottlenecks

**[PhysX Solver Overhead]:**
- Problem: The use of TGS (Task Graph Scheduler) and 8/4 iterations is balanced but might be slow for large numbers of environments.
- Files: `arcproLab/arcpro_env_cfg.py`.
- Cause: Required for stable 20kg dynamics at high frequency (200Hz).
- Improvement path: Experiment with reduced iterations or simplified collision hulls.

## Fragile Areas

**[Waypoint Precision]:**
- Files: `arcproLab/mdp/track_centerline.npy`.
- Why fragile: If the track USD is updated but the centerline is not re-generated, navigation will fail.
- Test coverage: `tests/test_track_manager.py` checks logic but not data validity.

**[USD External References]:**
- Files: `openStreetUSD/no_graph_sim.usd`.
- Why fragile: Historically pointed to absolute paths or missing assets (signposts).
- Mitigation: `repair_usd_references.py` redirects these to local placeholders.

## Scaling Limits

**[Hierarchical Bottleneck]:**
- Current capacity: Efficient path planning at 0.5m spacing.
- Limit: Very high-speed cornering where waypoint density may be insufficient for a 20kg vehicle.
- Scaling path: Dynamically adjust `num_waypoints` or `waypoint_horizon` based on vehicle speed.

## Missing Critical Features

**[Lane-Aligned Spawning]:**
- Problem: Jitter spawning with lateral and heading noise relative to waypoints is not yet implemented.
- Files: `arcproLab/mdp/events.py`.
- Blocks: Domain randomization for more robust policies.

## Test Coverage Gaps

**[4WD Telemetry Sync]:**
- What's not tested: Verification that all 4 wheels are contributing correctly to the `SPEED` observation.
- Files: `arcproLab/mdp/observations.py`, `arcproLab/mdp/rewards.py`.
- Risk: Incorrect speed calculation if only front/rear wheels are sampled.
- Priority: Medium.

---

*Concerns audit: 2025-04-05*
