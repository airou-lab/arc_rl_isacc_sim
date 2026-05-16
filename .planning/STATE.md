# Project State: ARCPro RL v2.6-mastery

## Current Milestone
**Milestone 3: HD Perception & Production Hardening** (ACTIVE)

## Current Phase
**Phase 16: MARL Transition** (ACTIVE)

## Summary
The project has successfully verified the **Mastery Curriculum** with 32 environments achieving 162 FPS. We have relaunched 32-env ResNet-18 training with a massive survival-first reward re-balance (terminating penalty weight 50.0, death = -25,000). Episode lengths have doubled to ~330-380 steps. Camera pipeline and GPU/VRAM stability are confirmed (10.6GB/12GB used on RTX 3060).

## Recent Activity
- **Mastery Training Relaunch**: ResNet-18 with survival-heavy rewards (Penalty: 50.0).
- **Episode Stability**: Average episode length increased from ~160 to ~350 steps.
- **Resource Audit**: Confirmed stable VRAM usage at 10.6GB for 32 envs with ResNet-18.
- **Infrastructure Refactor**: Vectorized `RoadManager` implemented and integrated into the telemetry observation stream.
- **Legacy Cleanup**: Removed `road_graph.py` and archived to `trash/`.

## Reference State (v16)
- **Model**: ResNet-18 (HD Vision)
- **Resolution**: 640x360
- **Environments**: 32 (Verified)
- **Reward Ratio**: Survival-first (Termination: 50.0, Speed: 10.0, Smoothness: 30.0)
- **Boundary Margin**: 0.05m (Mastery)
- **Infrastructure**: Vectorized RoadManager (B, N) support.

## Active Todos (Queue)
1. [x] **15-01-PLAN**: Implement parameterized boundary termination and mastery reward weights.
2. [x] **16-marl-transition**: Refactor singletons and establish multi-agent infrastructure.
3. [ ] **16-02-SKRL**: Integrate SKRL backend for multi-agent policy training. (NEXT)

**RESUME HERE**
- Milestone: Milestone 3
- Phase: Phase 16 (MARL Transition)
- Next Todo: Integrate SKRL backend and verify multi-agent environment configuration.
