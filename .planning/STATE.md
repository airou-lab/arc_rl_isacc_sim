# Project State: ARCPro RL v1.2-dev

## Current Milestone
**Milestone 1: Physical Fidelity & Policy Foundation**

## Current Phase
**Phase 5: Policy Integration**

## Summary
The project has successfully transitioned to "True Physics" mode (Phase 7). We are now focusing on completing Milestone 1 by integrating the Hierarchical Policy architecture, the 12-float telemetry protocol, and Gaussian-weighted rewards in the 1.0x metric environment. Phase 6 has been moved to Milestone 2 (v2.0) due to its architectural complexity.

## Recent Activity
- **Verification Restoration**: Restored `verify_policy.py` and supporting scripts to `arcproLab/scripts/`. ResNet18 baseline is ready for **USER TESTING**.
- **Phase 05-01 (ResNet18 Policy)**: **PENDING USER APPROVAL / NEEDS TESTING**. Road-following baseline needs verification in the 1.0x metric environment.
- **Phase 05-01 (Asset Verification)**: **STALE / REVISIT**. Investigated OSM "Original" tracks; identified "Invisible Barriers" at tile junctions. Current workaround is `no_graph_sim_final.usd`. Needs revisit to fix stitching.
- **Physical Stabilization**: Applied 3.5kg mass and contact offsets to the F1Tenth model. Developed a "Bake & Harden" workflow (`Flatten()` + `convexDecomposition`) for raw maps.
- **Visual Preservation**: Solved the "White Sheet" issue using `materialPurpose="physics"` for material binding.
- **Current Baseline**: Reverted to the stable `no_graph_sim_final.usd` at -1.25m offset for training smoothness.

## Key Achievements
- **True Physics Grounded**: Robot settles precisely at Z=0.0514m (matching wheel radius).
- **3D Navigation**: Confirmed `track_centerline.npy` contains 3D waypoints; navigation is altitude-aware.
- **Asset Archive**: Legacy and experimental USDs moved to `openStreetUSD/archive/` with cataloging.

## Active Tasks (Phase 5-02 - WAITING USER CONFIRMATION)
- [ ] Implement 12-Float Telemetry Protocol in `observations.py`.
- [ ] Implement 'Hybrid Racer' Gaussian rewards in `rewards.py`.
- [ ] Implement 'lane-aligned' randomized spawning in `events.py`.
- [ ] Integrate Hierarchical Policy stack from sibling repository.

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
