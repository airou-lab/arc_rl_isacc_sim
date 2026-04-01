# Project State: ARCPro RL v1.2-dev

## Current Milestone
**Milestone 1: Physical Fidelity & Policy Foundation**

## Current Phase
**Phase 5: Policy Integration**

## Summary
The project has successfully transitioned to "True Physics" mode (Phase 7). We are now focusing on completing Milestone 1 by integrating the Hierarchical Policy architecture, the 12-float telemetry protocol, and Gaussian-weighted rewards in the 1.0x metric environment. Plans for Phase 5-02 (Hierarchical Policy & Telemetry) have been created and incorporate user decisions on endless mode, curvature, odometry, ResNet18 fusion, and jitter spawning.

## Recent Activity
- **Phase 05-02 (Hierarchical Policy & Telemetry)**: **PLANNED**. Incorporates 3-point circle-fit curvature, 256-dim ResNet18 fusion, and randomized lane spawning.
- **Phase 05-02 (Refinement)**: Refined telemetry and policy stack plans for **Endless Lane Following**. Confirmed **FWD** configuration and **Worker node bypass**. Ready for Task 1 execution.
- **Verification Restoration**: Restored `verify_policy.py` and supporting scripts to `arcproLab/scripts/`. ResNet18 baseline is ready for **USER TESTING**.
- **Physical Stabilization**: Applied 3.5kg mass and contact offsets to the F1Tenth model. Developed a "Bake & Harden" workflow (`Flatten()` + `convexDecomposition`) for raw maps.
- **Visual Preservation**: Solved the "White Sheet" issue using `materialPurpose="physics"` for material binding.

## Key Achievements
- **True Physics Grounded**: Robot settles precisely at Z=0.0514m (matching wheel radius).
- **3D Navigation**: Confirmed `track_centerline.npy` contains 3D waypoints; navigation is altitude-aware.
- **Asset Archive**: Legacy and experimental USDs moved to `openStreetUSD/archive/` with cataloging.

## Active Tasks (Phase 5-02 - PLANNED)
- [ ] Implement 12-Float Telemetry Protocol (Index 2=0, Index 10=Kappa, Index 11=Odometry).
- [ ] Implement 3-point circle-fit calculation in `TrackManager`.
- [ ] Refactor `FusionFeaturesExtractor` to use ResNet18 (256-dim embedding).
- [ ] Implement 'lane-aligned' randomized spawning with lateral and heading noise in `events.py`.

## Planned Tasks
- [ ] **Phase 08: F1Tenth Physics Fidelity Restoration** - Restore damping/stiffness from original assets.
- [ ] **Phase 06: Intersection Navigation** - Implement graph-based route planning.

## Completed (Phase 7 - True Physics Mode)
- [x] Configure Environment Physics, Scaling, and Camera Offset in `arcpro_env_cfg.py`.
- [x] Configure Robot Asset in `arcpro_robot_cfg.py`.
- [x] Remove Robot Stability Event in `mdp/events.py`.
- [x] Correct and scale Track Centerline waypoints to Metric (0.0825 scale).
- [x] Update Observation/Telemetry protocol for Metric.
- [x] Verify True Physics Environment Setup (Human Verification).
