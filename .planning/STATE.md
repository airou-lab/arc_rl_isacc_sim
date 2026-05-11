# Project State: ARCPro RL v2.5-dev

## Current Milestone
**Milestone 2: Autonomous Urban Navigation** (ACTIVE)

## Current Phase
**Phase 15: HD Vision & ResNet Hardening** (ACTIVE)

## Summary
The project has successfully resolved the **Suicide Trap** and achieved **5M step convergence** on a 160x90 precision baseline. However, the model remains brittle in tight 6cm margins. We are now transitioning to **HD Vision (960x600)** using a **ResNet-18 backbone** and **Domain Randomization** to achieve robust, high-speed lapping.

## Recent Activity
- **Phase 15 Formulated**: Created `.planning/phases/15-hd-vision-resnet/` and established the HD ResNet implementation plan.
- **Precision Tuning Breakthrough**: Achieved >1,800 step survival in the 160x90 1.0x metric baseline at 50Hz.
- **Suicide Loop Break**: Successfully moved away from the 4-step collapse through centered spawning and reward rebalancing.

## Active Todos (Queue)
1. [x] **15-01-adaptive-cnn-backbone**: Implement Adaptive CNN with Early Pooling in `fusion_policy.py` for VRAM efficiency.
2. [x] **15-02-late-fusion**: Implement vision-dominant concatenation for HD-Telemetry integration.
3. [x] **15-03-hd-config**: Configure Isaac Lab for 640x360 rendering.
4. [ ] **15-04-production-hardening**: Launch 2M step hardening run with randomized spawns. (ACTIVE v7)

