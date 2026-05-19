# Project State: ARCPro RL v2.6-mastery

## Current Milestone
**Milestone 3: HD Perception & Production Hardening** (ACTIVE)

## Current Phase
**Phase 16: MARL Transition** (ACTIVE)

## Summary
The project has successfully verified the **Mastery Curriculum** with 32 environments achieving 162 FPS. We have relaunched 32-env ResNet-18 training with a massive survival-first reward re-balance (terminating penalty weight 50.0, death = -25,000). Episode lengths have doubled to ~330-380 steps. Camera pipeline and GPU/VRAM stability are confirmed (10.6GB/12GB used on RTX 3060).

## Recent Activity
- **Aggressive Mastery Relaunch**: Switched to Speed-first rewards (Speed Weight: 50.0, Termination: 10.0).
- **Behavior Intervention**: Identified "Cowardly Local Minimum" in v16-mastery (crawling at 0.05 mps); re-balanced to force progress.
- **Infrastructure Refactor**: Vectorized `RoadManager` implemented and integrated.

## Reference State (v16)
- **Model**: ResNet-18 (HD Vision)
- **Resolution**: 640x360
- **Environments**: 32
- **Reward Ratio**: Speed-first (Termination: 10.0, Speed: 50.0, Smoothness: 30.0)
- **Boundary Margin**: 0.05m (Mastery)

## Active Todos (Queue)
1. [x] **15-01-PLAN**: Implement parameterized boundary termination and mastery reward weights.
2. [x] **16-marl-transition**: Refactor singletons and establish multi-agent infrastructure.
3. [ ] **16-02-SKRL**: Integrate SKRL backend for multi-agent policy training. (NEXT)
4. [ ] **17-competitive-racing**: Overtaking and multi-agent interaction logic.
5. [ ] **18-env-hardening**: Re-introduce terrain, distractors, and domain randomization.

**RESUME HERE**
- Milestone: Milestone 3
- Phase: Phase 16 (MARL Transition)
- Next Todo: Integrate SKRL backend and verify multi-agent environment configuration.
