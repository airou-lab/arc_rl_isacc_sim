# Project State: ARCPro RL v2.9 (Hardened Mastery)

**RESUME HERE**
- **Milestone**: Milestone 3: HD Perception & Production Hardening
- **Phase**: Phase 16: MARL Transition
- **Next Todo**: Proceed with MARL Architecture Discussion. Focus on refactoring RoadGraph from singleton to agent-indexed tensors. Single-agent "Hardened Mastery" baseline is finalized.

---

## Current Milestone
**Milestone 3: HD Perception & Production Hardening** (ACTIVE)

## Current Phase
**Phase 16: MARL Transition** (ACTIVE)

## Summary
The project has reached the **Hardened Mastery v2.9** training baseline. This version focuses on extreme lateral precision and steering stability, suppressing the "jitter" observed in earlier v2.x iterations. Single-agent performance is now highly stable, providing a robust foundation for the upcoming Multi-Agent Reinforcement Learning (MARL) transition.

Key Stabilizations (Hardened Mastery v2.9):
- **Lateral Precision**: Shrinked safety plateau from 0.10m to **0.05m** and increased penalty slope to **10.0** in `lateral_error_reward`.
- **Steering Stabilization**: Increased `smoothness` reward weight to **15.0** to suppress traction-breaking jitters.
- **Exploration Balance**: Restored `ent_coef` to **0.01** to ensure the agent explores the now-stricter reward manifold.
- **Survival Metrics**: Initial Step 0 metrics show high survival (**199+ steps**) and stable convergence.

## Recent Activity
- **Reward Tuning**: Hardened the lateral error penalty and smoothness rewards to improve lane-centering and reduce jitter.
- **Hyperparameter Optimization**: Adjusted entropy coefficient for better exploration-exploitation balance under stricter constraints.
- **Training Relaunch**: Initiated a fresh 5M step training run to validate the hardened parameters.

## Reference State (v2.9)
- **Model**: ResNet-18 (Fusion Policy)
- **Scale**: 1.0x True Physics
- **Lateral Plateau**: 0.05m (Strict)
- **Smoothness Weight**: 15.0
- **Entropy Coef**: 0.01
- **Spawn Height**: 0.12m
- **Boundary Threshold**: 0.12m
- **Envs**: 32
- **Sensors**: Enabled (Camera 224x224)

## Active Todos (Queue)
1. [x] **16-01-REWARD-STABILITY**: Fix Greed Bug and Lazy Bureaucrat reward imbalances.
2. [x] **16-02-PROGRESS-REWARD**: Implement track-tangent projection for speed.
3. [x] **16-03-PHYSICS-STABILITY**: Confirm Gold Master orientation and drop logic.
4. [ ] **16-04-MARL-ARCHITECTURE**: Discuss refactoring RoadGraph to agent-indexed tensors.
