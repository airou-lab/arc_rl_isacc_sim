# Codebase Concerns

**Analysis Date:** 2025-04-18

## Tech Debt

**[Hybrid Scale Complexity]:**
- Issue: The simulation operates at **8.0x robot scale** but the policy expects **1.0x metric scale**. This requires manual normalization (`* 0.125`) in `observations.py`.
- Files: `arcproLab/arcpro_env_cfg.py`, `arcproLab/mdp/observations.py`.
- Impact: Increased cognitive load for developers and potential for scaling bugs if new telemetry fields are added without normalization.
- Fix approach: Re-export the track at true 1.0x scale to allow the robot to return to its canonical 1.0x scale.

**[Hardcoded Initial State]:**
- Issue: The robot's spawn position is hardcoded in `arcpro_env_cfg.py`.
- Files: `arcproLab/arcpro_env_cfg.py`.
- Impact: Training always starts from the same point, reducing exploration diversity.
- Fix approach: Implement a robust `reset_robot_to_lane` event in `mdp/events.py` that samples random waypoints.

## Known Bugs

**[USD Sampling Jitter]:**
- Issue: `TrackManager.sample_waypoints_from_usd` uses a nearest-neighbor approach that can fail on complex intersection geometry.
- Files: `arcproLab/mdp/track_manager.py`.
- Trigger: Multiple overlapping road meshes in USD.
- Workaround: Use pre-computed `track_centerline.npy` instead of live sampling.

## Security Considerations

**[Local Simulation]:**
- Risk: None identified; simulation runs locally with standard dependencies.

## Performance Bottlenecks

**[GPU Memory Usage]:**
- Problem: Isaac Sim 4.0 with large USD files can consume significant VRAM.
- Files: `openStreetUSD/no_graph_sim.usd`.
- Cause: High-detail road meshes.
- Improvement path: Optimize collision geometry and use lower-poly visual models where appropriate.

## Fragile Areas

**[Waypoint-Track Alignment]:**
- Files: `arcproLab/mdp/track_centerline.npy`.
- Why fragile: If the track is moved in the USD stage, the waypoints will no longer align.
- Test coverage: Low.
- Safe modification: Always run `verify_metric.py` after updating track assets.

## Scaling Limits

**[Multi-Agent Support]:**
- Current capacity: Single robot.
- Limit: `TrackManager` is a singleton, which may need refactoring if multiple robots need to track different segments simultaneously (though the logic is vectorized).

## Missing Critical Features

**[Domain Randomization]:**
- Problem: Friction, mass, and lighting randomization are not yet implemented.
- Blocks: Robust sim-to-real transfer.

## Test Coverage Gaps

**[4WD Force Verification]:**
- What's not tested: Whether all 4 wheels are actually applying the correct torque.
- Files: `arcproLab/arcpro_robot_cfg.py`, `arcproLab/mdp/observations.py`.
- Risk: Robot might be operating as 2WD due to misconfiguration, reducing its performance under high load.
- Priority: Medium.

---

*Concerns audit: 2025-04-18*
