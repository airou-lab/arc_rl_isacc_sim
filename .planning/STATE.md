# Project State: ARCPro RL v2.9-gold-master

**RESUME HERE**
- **Milestone**: Milestone 3
- **Phase**: Phase 16 (MARL Transition)
- **Next Todo**: Proceed with MARL Architecture Discussion and multi-agent refactoring. Single-agent baseline is now "Gold Master" stabilized. Focus on refactoring RoadGraph from singleton to agent-indexed tensors.

---

## Current Milestone
**Milestone 3: HD Perception & Production Hardening** (ACTIVE)

## Current Phase
**Phase 16: MARL Transition** (ACTIVE)

## Summary
The project has reached the "Gold Master" training baseline for Phase 16. We have successfully stabilized the single-agent physics, reward mathematics, and environment boundary logic. A fresh Step 0 training run is currently active with high survival metrics and stable convergence.

Key Stabilizations (Gold Master):
- **Ground Settle Logic**: Mandatory action lock (zero throttle/steer) until chassis drops below Z=0.10m, ensuring suspension stability.
- **Physics Fidelity**: North-facing spawn, upright orientation (180-roll), and 12cm drop height confirmed as the production baseline.
- **Boundary Precision**: Strict 0.12m paint-contact threshold restored for high-precision lane following.
- **Intersection Permeability**: Implemented 1.0m "Hole Punching" in boundary markers at lane gates to allow seamless intersection navigation.

## Recent Activity (Gold Master Launch)
- **Environment Hardening**: Implemented Hole Punching in `TrackManager` and Action Locking in `WaypointTrackingWrapper`.
- **Training Reset**: Wiped legacy checkpoints and initiated a fresh PPO run from Step 0.
- **Metric Verification**: Confirmed average survival of 190 steps and 0.22 m/s cruise speed in early Step 0 logs.

## Reference State (v2.9)
- **Model**: ResNet-18 (Fusion Policy)
- **Scale**: 1.0x True Physics
- **Orientation**: Upright (180-roll)
- **Drive Polarity**: `scale=-20.0` (Native drive scaling)
- **Spawn Height**: 0.12m (Drop height)
- **Settling Threshold**: 0.10m (Action lock release)
- **Boundary Threshold**: 0.12m
- **Envs**: 32
- **Sensors**: Enabled (Camera 224x224)

## Active Todos (Queue)
1. [x] **16-01-REWARD-STABILITY**: Fix Greed Bug and Lazy Bureaucrat reward imbalances.
2. [x] **16-02-PROGRESS-REWARD**: Implement track-tangent projection for speed.
3. [x] **16-03-PHYSICS-STABILITY**: Confirm Gold Master orientation and drop logic.
4. [ ] **16-04-MARL-ARCHITECTURE**: Discuss refactoring RoadGraph to agent-indexed tensors.
