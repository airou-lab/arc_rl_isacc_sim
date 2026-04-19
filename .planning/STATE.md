# Project State: ARCPro RL v1.2-dev

## Current Milestone
**Milestone 1: Physical Fidelity & Policy Foundation** (ACTIVE)

## Current Phase
**Phase 11: Retraining & Intersection Logic** (ACTIVE - Integration Planning)

## Summary
Achieved True 1.0x Metric Scale. Completed initial PPO retraining which identified a critical "Silent Failure" in lateral error perception (fixed in TrackManager). Now transitioning to the Hierarchical Path Planning Policy (HPPP) to provide waypoint-based navigation and robust visual feature extraction.

## Recent Activity
- **Silent Failure Root Cause**: Identified that `TrackManager.compute_errors` was a legacy stub returning `0.0`.
- **Perception Fix**: Implemented real lateral error calculation in `TrackManager` using USD markers.
- **Brain Selection**: Decided to integrate `HierarchicalPathPlanningPolicy` from reference stack.
- **Integration Research**: Mapped telemetry vector Protocol v2 to align with HPPP expectations.

## Active Todos (Queue)
1. [ ] **11-02-obs-alignment**: Align `mdp/observations.py` with HPPP Protocol v2.
2. [ ] **11-03-brake-action**: Implement `CombinedDriveAction` in `mdp/actions.py`.
3. [ ] **11-05-file-sync**: Import HPPP and FusionExtractor into `policy_stack/`.
4. [ ] **11-06-train-setup**: Update `train_policy.py` for RecurrentPPO + Auxiliary losses.

## Completed
- [x] Phase 11: 1.0x Baseline Retraining (Identified perception bugs).
- [x] Phase 09: Training Loop Stabilization (8x Baseline).
- [x] Phase 10: Asset Downscaling (1.0x Restoration).

## Blockers
None.
