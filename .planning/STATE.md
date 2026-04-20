# Project State: ARCPro RL v1.2-dev

## Current Milestone
**Milestone 1: Physical Fidelity & Policy Foundation** (ACTIVE)

## Current Phase
**Phase 11: Retraining & Intersection Logic** (ACTIVE - Integration Planning)

## Summary
Achieved True 1.0x Metric Scale. Completed initial PPO retraining which identified a critical "Silent Failure" in lateral error perception (fixed in TrackManager). Now transitioning to the Hierarchical Path Planning Policy (HPPP) to provide waypoint-based navigation and robust visual feature extraction.

## Recent Activity
- **Spawn Alignment Fix**: Resolved issue where robot spawned on road divider (double yellow line). Shifted spawn to lane center (X ~ -16.18) for stable episode starts.
- **Orientation Fix**: Corrected South-facing quaternion (WXYZ) in `events.py` and `arcpro_env_cfg.py` to prevent tilted/incorrect robot pose.
- **Perception Offset**: Added `target_lane_offset` to `TrackManager` to allow policy to track lane center instead of road center.
- **Brain Integration**: Successfully verified Hierarchical Path Planning Policy (HPPP) with real telemetry in GUI rollouts.

## Active Todos (Queue)
1. [/] **11-09-production-run-v2**: (RUNNING) Retrain HierarchicalPathPlanningPolicy for 5,000,000 steps with fixed camera tilt and dynamic heading.
   - Command: `DISPLAY=:0 bash train.sh --num_envs 32 --headless`
   - Log: `training_5m_v2.log`
   - Fixes: 20deg camera tilt down, dynamic heading error calculation.

## Completed
- [x] Phase 11: 1.0x Baseline Retraining & Perception Fixes.
- [x] Phase 11: Hierarchical Policy Integration (Submodule, Protocol v2, Bridges).
- [x] Phase 11: Core Perception Debug (Camera tilt/Heading math).

## Blockers
None.
