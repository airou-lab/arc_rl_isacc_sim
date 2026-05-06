# Project State: ARCPro RL v2.5-dev

## Current Milestone
**Milestone 3: Real-Time Visualization & Diagnostics** (ACTIVE)

## Current Phase
**Phase 14: Multi-Agent Environment Refactor (Scaling)** (ACTIVE)

## Summary
Successfully transitioned to a natively 1.0x metric USD asset (Phase 14-01). Resolved the "trap at spawn" termination bug by deleting the stale boundary cache and relaxing the threshold to 0.12m. Verified with a 16-env smoke test (episodes > 300 steps). Physics (5kg) and DT (20Hz) are fully stabilized. We are now scaling to 32-env production training (5M timesteps).

## Recent Activity
- **Phase 14-01 USD Flattening**: Successfully migrated to physically scaled 1.0x USD asset and refreshed boundary cache.
- **Phase 12 Implementation**: Verified random gate selection per episode for intersection decision-making. Awaiting confirmation from trained policy.
- **Bug Fix (2026-05-06)**: Resolved immediate training termination by refreshing stale boundary cache.
- **Production Training**: Launching 32-env scaling.

## Active Todos (Queue)
1. [ ] **32-env-scaling**: (ACTIVE) Run production training with 32 environments (Target: 5M timesteps).
2. [ ] **12-01-turning-mission**: (AWAITING CONFIRMATION) Random gate selection implemented; verify with policy once training completes.
3. [ ] **14-01-multi-agent-scaffolding**: Refactor `ManagerBasedRLEnv` to support N robots interacting in the *same* environment instance.


## Completed
- [x] Phase 14-01: USD Asset Flattening (1.0x Metric Physical Scale).
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
