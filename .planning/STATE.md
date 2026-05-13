# Project State: ARCPro RL v2.5-dev

## Current Milestone
**Milestone 3: HD Perception & Production Hardening** (ACTIVE)

## Current Phase
**Phase 15: HD Vision & Adaptive CNN** (ACTIVE)

## Summary
The project has successfully resolved the **Suicide Trap** and achieved **5M step convergence** on a 160x90 precision baseline. We have transitioned to **HD Vision (640x360)** using a custom **Adaptive CNN backbone** with early pooling to maintain VRAM stability on the RTX 3060. Production training is currently focused on hardening the policy for 6cm clearance margins.

## Recent Activity
- **VRAM Stabilization**: Discovered that 30 parallel HD renders caused process termination. Reduced environment count to **16** to achieve stable ~6.2GB VRAM usage.
- **Backbone Optimization**: Implemented `AdaptiveAvgPool2d((128, 128))` in `fusion_policy.py` to decouple input resolution from CNN connection density.
- **Production Run (v14)**: Launched a 5M step hardening run with 16 environments, 128x128 resolution (via Adaptive Pooling), and 30.0 smoothness reward weight.

## Reference State (v14)
- **Resolution**: 128x128 (via AdaptiveAvgPool2d)
- **Environments**: 16 parallel envs
- **Smoothness Weight**: 30.0
- **Status**: Stable background training active

## Active Todos (Queue)
1. [x] **15-01-adaptive-cnn-backbone**: Implement Adaptive CNN with Early Pooling in `fusion_policy.py` for VRAM efficiency.
2. [x] **15-02-late-fusion**: Implement vision-dominant concatenation for HD-Telemetry integration.
3. [x] **15-03-hd-config**: Configure Isaac Lab for 640x360 rendering.
4. [ ] **15-04-production-hardening**: ACHIEVE 5M steps with randomized spawns and zero target offset. (ACTIVE)
5. [ ] **15-05-mastery-verification**: Run GUI verification once training exceeds 1M steps to confirm centerline mastery.

**RESUME HERE**
- Milestone: Milestone 3
- Phase: Phase 15
- Next Todo: Monitor training progress for 5M step milestone.
