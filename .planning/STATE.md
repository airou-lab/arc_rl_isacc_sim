# Project State: ARCPro RL v2.8-marl-stabilized

**RESUME HERE**
- **Milestone**: Milestone 3
- **Phase**: Phase 16 (MARL Transition)
- **Next Todo**: Proceed with MARL Architecture Discussion and training. Single-agent physics and rewards are stabilized. Next: Expand to multi-agent HD vision with Adaptive CNN. Focus on refactoring RoadGraph from singleton to agent-indexed tensors.

---

## Current Milestone
**Milestone 3: HD Perception & Production Hardening** (ACTIVE)

## Current Phase
**Phase 16: MARL Transition** (ACTIVE)

## Summary
Phase 16 single-agent training has reached "Step 0" stability. We have resolved the mathematical imbalance in rewards and the physics instability of the F1Tenth asset. The agent now shows healthy convergence patterns (decreasing entropy, stable variance) thanks to the Progress-Based Speed reward and the Control Flip strategy.

Key Stabilizations:
- **Progress-Based Speed**: Rewards are now tied to actual forward progress along the track tangent, preventing "sliding" exploits.
- **Reward Hierarchy**: Balanced incentives so that Full Lap (+2750) > Death (-1000) > Stagnation (-100/step).
- **Physics Stabilization**: Native upside-down USD spawn orientation with a reversed drive scale (-20.0) is confirmed as the stable baseline.
- **Rolling Start**: Implemented a 0.2-0.5 m/s rolling start to assist the 5kg chassis in overcoming initial inertia without triggering stagnation limits.

## Recent Activity (Phase 16 Stabilization)
- **Reward Tuning**: Fixed "Greed Bug" (Sprint and Die) and "Lazy Bureaucrat" (Crawl and Do Nothing) by rebalancing weights.
- **Progress Reward**: Implemented track-tangent projection in `rewards.py`.
- **Physics Confirmation**: Validated "Control Flip" as the necessary stable state for F1Tenth suspension.
- **Training Baseline**: Fresh Step 0 run shows healthy convergence.

## Reference State (v2.8)
- **Model**: ResNet-18 (Fusion Policy)
- **Scale**: 1.0x True Physics
- **Orientation**: Native Upside-Down (WXYZ math: `W=cos(half_yaw), X=0.0, Y=0.0, Z=sin(half_yaw)`)
- **Drive Polarity**: `scale=-20.0` (Flipped to compensate for upside-down chassis)
- **Spawn Height**: 0.12m
- **Damping**: 1.0 (Throttle) / 2.0 (Steering)
- **Envs**: 32
- **Sensors**: Enabled (Camera 224x224)

## Active Todos (Queue)
1. [x] **16-01-REWARD-STABILITY**: Fix Greed Bug and Lazy Bureaucrat reward imbalances.
2. [x] **16-02-PROGRESS-REWARD**: Implement track-tangent projection for speed.
3. [x] **16-03-PHYSICS-STABILITY**: Confirm Control Flip and orientation for F1Tenth.
4. [ ] **16-04-MARL-ARCHITECTURE**: Discuss refactoring RoadGraph to agent-indexed tensors.
