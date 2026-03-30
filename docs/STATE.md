# Project State: ARCPro RL v1.2-dev

## Current Phase
**Phase 7: Revert to True Physics Mode**

## Summary
The project has successfully transitioned to "True Physics" mode. We have reverted the simulation to 1.0x metric scaling for both the robot (`F1Tenth_Metric.usd`) and the track (0.0825 scale). The road surface is correctly grounded at Z=0 using a -1.25m offset. All previous workarounds (Giant Scale, stability events, high iterations) have been removed.

## Recent Activity
- **Metric Grounding**: Applied `0.0825` scale to the track and `-1.25m` Z-offset to ground the surface at Z=0.
- **True Physics Config**: Updated `arcpro_env_cfg.py` and `arcpro_robot_cfg.py` to use 1.0x metric assets and stable solver iterations (8, 4).
- **Event Cleanup**: Removed the `setup_robot_stability` "Hard Override" from `mdp/events.py` as it's no longer necessary.
- **Metric Centerline**: Scaled `track_centerline.npy` by `0.0825` to align waypoint errors with the new metric environment.
- **Planning Reorganization**: Cleaned up the `.planning/` directory, moving old phases to milestones and centralizing research.

## Key Achievements
- **True Physics Grounded**: Simulation operates at a realistic 1.0x metric scale with standard PhysX settings.
- **Metric Telemetry**: Confirmed that lateral/heading errors and speeds are correctly calculated in meters/seconds.
- **Stability Verified**: Robot remains stable at ground-level without previous high-iteration/damping workarounds.

## Active Tasks (Phase 7)
- [x] Configure Environment Physics, Scaling, and Camera Offset in `arcpro_env_cfg.py`.
- [x] Configure Robot Asset in `arcpro_robot_cfg.py`.
- [x] Remove Robot Stability Event in `mdp/events.py`.
- [x] Correct and scale Track Centerline waypoints to Metric (0.0825 scale).
- [x] Update Observation/Telemetry protocol for Metric.
- [x] Verify True Physics Environment Setup (Human Verification).

## Backlog (Phase 5 - Policy Integration)
- [ ] Implement 'lane-aligned' spawning logic in `events.py`.
- [ ] Implement 12-float telemetry protocol in `observations.py`.
- [ ] Implement 'Hybrid Racer' Gaussian rewards in `rewards.py`.
- [ ] Integrate Hierarchical Path Planning Policy components from `arc_rl_isacc_policy`.
