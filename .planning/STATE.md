# Project State: ARCPro RL v2.5-dev

## Current Milestone
**Milestone 3: Real-Time Visualization & Diagnostics** (ACTIVE)

## Current Phase
**Phase 14-01: Procedural Multi-Agent Scaffolding** (ACTIVE)

## Summary
The project has successfully resolved the **Fatal Coordinate Mismatch** that caused previous policy collapses. Measurement confirmed the 1.0x metric lane is only **0.407m wide**, meaning the previous training target (-0.238m offset) was physically **3cm inside the white line boundary**. This caused immediate self-destruction upon loading. We have now standardized to a consistent **Right-Handed Coordinate Frame (Positive-Left for both Steer and LatErr)** and neutralized the target to **0.0m (Centerline)**. 

## Recent Activity
- **Fatal Bias Resolution**: Reset `target_lane_offset` to 0.0m after measuring actual 0.407m lane width.
- **Coordinate Frame Sync**: Standardized `TrackManager` and `AckermannSteering` to **Positive-Left** (Right-Handed).
- **Physics Stabilization**: Fixed ground clipping by setting spawn height to 0.10m and explicitly setting **Earth Gravity (-9.81)**.
- **Termination Logic**: Implemented 10-step spawn grace period and increased boundary resolution to 1cm.
- **Health Fix**: Generated missing `VALIDATION.md` files for Phases 11-14 and standardizing all phase documentation.

## Active Todos (Queue)
1. [ ] **14-01-production-training**: (ACTIVE) Complete 5M steps on 32 envs with neutral, synchronized physics.
2. [ ] **14-01-01-marl-config**: Create `arcpro_marl_env_cfg.py` with loop-based N-agent support.
3. [ ] **14-01-03-roadgraph-2d**: Upgrade `RoadGraph` to handle per-agent navigation tokens.
4. [ ] **14-01-04-collision-global-reset**: Implement inter-agent crash detection and global environment reset.

## Completed
- [x] Phase 14-01: Multi-Agent Research & Strategy (V2X Focus).
- [x] Phase 14-01: USD Asset Flattening (1.0x Metric Physical Scale).
- [x] Phase 11: 1.0x Baseline Retraining & Perception Fixes.

## Health Status (2026-05-08)
- **Status**: **Healthy**
- **Summary**: All validation docs generated, frontmatter repaired, and stale directories archived. Training is stable.
