# Project State

## Current Phase: 5.0 (Training & Policy Development)
## Status: Environment Stabilized - Proportion Calibration Complete

## Completed Tasks (Phase 4: Robot Refinement)
- [x] M004/S01: Robot Scaling & Curb Weight (4.092kg).
- [x] M004/S02: RGB Camera Configuration (160x90).
- [x] M004/S03: Viewer Tracking & Free Camera.
- [x] M004/S04: Clipping Check & Physics Hardening.

## Completed Tasks (Phase 7: Surgical Calibration)
- [x] M007/S01: Proportional Calibration (1.0x Scale Alignment).
- [x] M007/S02: Environment Reconstruction (Based on `no_graph_sim_cleaned.usd`).
- [x] M007/S03: Clean Asset Integration (Removed conflicting embedded robots/graphs).

## Current Tasks (Phase 5: Policy Development)
- [x] M005/S01: Link original SB3 policy with Isaac Lab environment (PolicyWrapper).
- [x] M005/S02: Implement action logic (coordinate to joint actions).
- [x] M005/S03: Create inference script (verify_policy.py).
- [ ] M005/S04: Resolve 3D Mesh Clipping (Robot currently drops through map meshes).
- [ ] M005/S05: Perform full-lap inference verification.

## Future Tasks (Phase 6: Navigation)
- [ ] M006/S01: Graph-Based Track Management for Intersections.
- [ ] M006/S02: High-level Navigation Command Implementation.
