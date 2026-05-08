# Project State: ARCPro RL v2.5-dev

## Current Milestone
**Milestone 3: Real-Time Visualization & Diagnostics** (ACTIVE)

## Current Phase
**Phase 14-01: Procedural Multi-Agent Scaffolding** (ACTIVE)

## Summary
The project has successfully stabilized single-agent 1.0x metric physics and is now transitioning to **Phase 14-01: Procedural Multi-Agent Scaffolding**. Recent training runs show significant progress with an average episode length (EpLen) of ~83 steps at the ~125k step mark, indicating stable tracking and intersection approach behavior. The core strategy for multi-agent interaction is the **Rough Placement V2X Strategy**, which prioritizes agent proximity at intersections to accelerate coordination learning.

## Recent Activity
- **Physics Stabilization**: Fixed "invisible bumps" by reducing CoM offset to -1cm.
- **Steering Bias Correction**: Neutralized `target_lane_offset` to 0.0m to stop rightward drift.
- **Termination Logic**: Added yellow line contact detection to `white_line_contact`.
- **Torque Calibration**: Lowered `drive_scale` to 8.0 for the 5kg chassis.
- **Production Training**: Restarted 32-env training with stabilized 1.0x parameters.

## Active Todos (Queue)
1. [ ] **14-01-production-training**: (ACTIVE) Complete 5M steps on 32 envs with fixed physics.
2. [ ] **14-01-01-marl-config**: Create `arcpro_marl_env_cfg.py` with loop-based N-agent support.
3. [ ] **14-01-03-roadgraph-2d**: Upgrade `RoadGraph` to handle per-agent navigation tokens.
3. [ ] **14-01-04-collision-global-reset**: Implement inter-agent crash detection and global environment reset.

## Completed
- [x] Phase 14-01: Multi-Agent Research & Strategy (V2X Focus).
- [x] Phase 14-01: USD Asset Flattening (1.0x Metric Physical Scale).
- [x] Phase 11: 1.0x Baseline Retraining & Perception Fixes.

## Health Status (2026-05-08)
- **Status**: Degraded.
- **Issues**:
  - Missing VALIDATION.md for Phases 11, 12, 13.
  - Missing 'wave' frontmatter in Phase PLAN files.
  - Missing SUMMARY.md for several sub-plans.
- **Remediation**: Re-run planning phases with --research to regenerate validation architectures or manually create VALIDATION.md files.
