# Project State: ARCPro RL v1.2-dev

## Current Milestone
**Milestone 1: Physical Fidelity & Policy Foundation** (ACTIVE)

## Current Phase
**Phase 11: Retraining & Intersection Logic** (ACTIVE - Integration Planning)

## Summary
Achieved True 1.0x Metric Scale. Completed initial PPO retraining which identified a critical "Silent Failure" in lateral error perception (fixed in TrackManager). Now transitioning to the Hierarchical Path Planning Policy (HPPP) to provide waypoint-based navigation and robust visual feature extraction.

## Recent Activity
- **Ground-Normal Alignment**: Implemented raycast-based orientation matching in `events.py`. Robot now spawns flush with the terrain, eliminating physics instabilities.
- **Policy Divergence Fix**: Identified that resuming from checkpoints was ignoring our `5e-5` learning rate and using the old `3e-4` value, causing `approx_kl` spikes (8.05). Added an explicit override in `train_policy.py`.
- **Robust Termination**: Implemented "Dense Wall" marker interpolation in `TrackManager` and tuned threshold to 0.15m to prevent leakage.
- **Diagnostic Tooling**: Added `test_straight_line.py` to verify boundary resets independently of the policy.

## Active Todos (Queue)
1. [/] **11-14-production-run-v8**: (PAUSED) Retraining HPPP.
2. [ ] **11-15-wave-2-intersections**: (IN PROGRESS) Ground-locking fixed, but intersection classification and path continuity still need work.
   - Status: Cyan spheres height fix applied; markers now dense (0.05m).
3. [ ] **11-16-road-graph**: Implement `RoadGraph` for multi-segment navigation.

## Active Issues
None.

## Completed
- [x] Phase 11: 1.0x Baseline Retraining & Perception Fixes.
- [x] Phase 11: Hierarchical Policy Integration (Submodule, Protocol v2, Bridges).
- [x] Phase 11: Core Perception Debug (Camera tilt/Heading math).

## Blockers
None.
