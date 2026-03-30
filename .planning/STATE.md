# Project State: ARCPro RL v1.2-dev

## Current Phase
**Phase 5: Policy Integration**

## Summary
The project has successfully transitioned to "True Physics" mode (Phase 7). We are now resuming Phase 5 to integrate the Hierarchical Policy architecture, the 12-float telemetry protocol, and Gaussian-weighted rewards in the 1.0x metric environment.

## Recent Activity
- **Phase 7 Completion**: Reverted the simulation to 1.0x metric scaling for both the robot and the track. The road surface is grounded at Z=0 using a -1.25m offset.
- **Metric Grounding**: Applied `0.0825` scale to the track and `-1.25m` Z-offset to ground the surface at Z=0.
- **Telemetry Alignment**: Scaled `track_centerline.npy` by `0.0825` to align waypoint errors with the new metric environment.
- **Hierarchical Planning**: Created `05-02-PLAN.md` to integrate the full hierarchical policy stack and 12-float protocol.

## Key Achievements
- **True Physics Grounded**: Simulation operates at a realistic 1.0x metric scale with standard PhysX settings.
- **Metric Telemetry**: Confirmed that lateral/heading errors and speeds are correctly calculated in meters/seconds.
- **Main Merge**: Successfully merged all "True Physics" changes into the `main` branch.

## Active Tasks (Phase 5)
- [ ] Implement 12-Float Telemetry Protocol in `observations.py`.
- [ ] Implement 'Hybrid Racer' Gaussian rewards in `rewards.py`.
- [ ] Implement 'lane-aligned' spawning logic in `events.py`.
- [ ] Integrate Hierarchical Path Planning Policy components from `arc_rl_isacc_policy`.
- [ ] Verify full stack at 1.0x metric scale using `verify_policy.py`.

## Completed (Phase 7 - True Physics Mode)
- [x] Configure Environment Physics, Scaling, and Camera Offset in `arcpro_env_cfg.py`.
- [x] Configure Robot Asset in `arcpro_robot_cfg.py`.
- [x] Remove Robot Stability Event in `mdp/events.py`.
- [x] Correct and scale Track Centerline waypoints to Metric (0.0825 scale).
- [x] Update Observation/Telemetry protocol for Metric.
- [x] Verify True Physics Environment Setup (Human Verification).
