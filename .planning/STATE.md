# Project State: ARCPro RL v1.2-dev

## Current Milestone
**Milestone 1: Physical Fidelity & Policy Foundation** (ACTIVE)

## Current Phase
**Phase 13: Live Policy GUI** (ACTIVE)

## Summary
Completed Phase 11 Intersection Logic (Wave 2) and researched Phase 12 Decentralized V2I. We are now shifting focus to Phase 13 (Live Policy GUI) to visualize live rewards, learning over time, and policy state (action heatmaps) via a shared memory sidecar GUI, before executing the Phase 12 decentralized turning missions.

## Recent Activity
- **Phase 11 Completion**: Finalized Wave 2 Intersection Navigation (Permeable gates, Intent filtering).
- **Phase 12 Research**: Documented decentralized V2I architecture (Edge Modules) and Sim2Real constraints.
- **Phase 13 Research**: Outlined the architecture for a Shared Memory Sidecar GUI (PyQt5/OpenCV) to render policy heatmaps and live reward graphs without blocking the 115 FPS simulation loop.

## Active Todos (Queue)
1. [ ] **13-01-shared-memory-bridge**: (NEXT) Create the shared memory data pipe in `visual_analytics.py` for frames, rewards, and distributions.
2. [ ] **13-02-policy-dashboard**: (PLANNED) Build `policy_dashboard.py` to render the Graph View.
3. [ ] **12-01-turning-mission**: (PAUSED) Execute first autonomous turn via RoadGraph token.
4. [ ] **update-log-paths**: (PLANNED) Update script file paths to route all new `.log` outputs to the root `logs/` directory.
5. [ ] **14-01-usd-asset-flattening**: (PLANNED) Remove the 0.125 scaling hack from the USD track asset.
6. [ ] **14-02-trackmanager-caching**: (PLANNED) Cache TrackManager boundary points to .npy files to fix startup performance bottleneck.
7. [ ] **14-03-telemetry-curvature**: (PLANNED) Implement path curvature (Kappa) calculation for the observation vector.

## Active Issues
None.

## Completed
- [x] Phase 11: 1.0x Baseline Retraining & Perception Fixes.
- [x] Phase 11: Hierarchical Policy Integration (Submodule, Protocol v2, Bridges).
- [x] Phase 11: Core Perception Debug (Camera tilt/Heading math).
- [x] Phase 11: Wave 2 Intersection Navigation (Permeable gates, Intent filtering).

## Blockers
None.
