# Project State: ARCPro RL v1.2-dev

## Current Milestone
**Milestone 1: Physical Fidelity & Policy Foundation** (ACTIVE)

## Current Phase
**Phase 13: Live Policy GUI** (ACTIVE)

## Summary
Completed Phase 11 Intersection Logic (Wave 2) and researched Phase 12 Decentralized V2I. We are now shifting focus to Phase 13 (Live Policy GUI) to visualize live rewards, learning over time, and policy state (action heatmaps) via a shared memory sidecar GUI, before executing the Phase 12 decentralized turning missions.

## Recent Activity
- **Phase 09 Completion**: Successfully migrated to 5kg realistic physics at 1.0x metric scale. Resolved spawn-death loops and wheel 'explosions'.
- **Bug Fix (2026-05-05)**: Resolved critical "stuck robot" issue. Fixed `WaypointTrackingWrapper` to correctly use the `policy` observation key for dead-reckoning. 
- **Production Training**: 16-env production run is currently active with the fixed wrapper.
- **Spawn Tuning**: Set stable spawn Z-offset to **0.1m** and height termination to **-0.5m**.
- **Phase 11 Completion**: Finalized Wave 2 Intersection Navigation (Permeable gates, Intent filtering).

## Active Todos (Queue)
1. [ ] **12-01-dt-alignment**: (CRITICAL) Resolve the mismatch between `ARCProEnvCfg` (20Hz) and `WaypointTrackingWrapper` (50Hz) to fix hierarchical loss degradation.
2. [ ] **12-01-turning-mission**: (NEXT) Implement the proximity-based RoadGraph triggers for intersection decision-making.
3. [ ] **13-01-shared-memory-bridge**: (PAUSED) Create the shared memory data pipe in `visual_analytics.py`.
4. [ ] **13-02-policy-dashboard**: (PAUSED) Build `policy_dashboard.py` to render the Graph View.
5. [ ] **14-01-usd-asset-flattening**: (PAUSED) Remove the 0.125 scaling hack from the USD track asset (waiting for training run to finish).
6. [ ] **update-log-paths**: (COMPLETED) All script log paths updated to `logs/`.

## Completed
- [x] Phase 11: 1.0x Baseline Retraining & Perception Fixes.
- [x] Phase 11: Hierarchical Policy Integration (Submodule, Protocol v2, Bridges).
- [x] Phase 11: Core Perception Debug (Camera tilt/Heading math).
- [x] Phase 11: Wave 2 Intersection Navigation (Permeable gates, Intent filtering).
- [x] Maintenance: 14-02-trackmanager-caching (Verified).
- [x] Maintenance: 14-03-telemetry-curvature (Verified).

## Active Issues
None.

## Completed
- [x] Phase 11: 1.0x Baseline Retraining & Perception Fixes.
- [x] Phase 11: Hierarchical Policy Integration (Submodule, Protocol v2, Bridges).
- [x] Phase 11: Core Perception Debug (Camera tilt/Heading math).
- [x] Phase 11: Wave 2 Intersection Navigation (Permeable gates, Intent filtering).

## Blockers
None.
