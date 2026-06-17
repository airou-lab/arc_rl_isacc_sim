# Project State: ARCPro RL v2.9 (Hardened Mastery / Gold Master)

**RESUME HERE**
- **Milestone**: Milestone 3: HD Perception & Production Hardening
- **Phase**: Phase 16: MARL Transition
- **Next Todo**: Proceed with MARL Architecture Discussion. Focus on refactoring RoadGraph from singleton to agent-indexed tensors and resolving VRAM constraints for multi-agent HD vision (Adaptive CNN).

---

## Current Milestone
**Milestone 3: HD Perception & Production Hardening** (ACTIVE)

## Current Phase
**Phase 16: MARL Transition** (ACTIVE)

## Summary
The project has reached the **Gold Master** training baseline for Phase 16, building upon the Hardened Mastery v2.9 state. Single-agent performance is highly stable, with critical mechanical and physical bugs resolved, providing a robust foundation for the upcoming Multi-Agent Reinforcement Learning (MARL) transition. The final Phase 16 training run is actively surviving and monitored.

Key Stabilizations (Gold Master Finalization):
- **Upright Physics & Settle**: Solved suspension-bounce reset by lowering spawn height to 12cm and height termination threshold to 2cm.
- **Ground Settle Logic**: Intercepted actions in `WaypointTrackingWrapper`, forcing 0.0 actions until robot Z height < 0.10m to prevent mid-air wheel spin.
- **Vision FOV**: Camera re-mounted `pos=(0.3, 0.0, 0.35)` and tilted 30 degrees down to ensure full track visibility.
- **Lateral Precision**: Shrinked safety plateau from 0.10m to **0.05m** and increased penalty slope to **10.0** in `lateral_error_reward`.
- **Steering Stabilization**: Increased `smoothness` reward weight to **15.0** to suppress traction-breaking jitters.

## Recent Activity
- **Physics Debugging**: Solved false-positive base-height terminations and explosions upon spawn.
- **Camera Calibration**: Adjusted camera orientation to solve FOV blind spots.
- **Training Launch**: Launched Phase 16 "Gold Master" training from Step 0, immediately achieving ~177 steps of survival. Active watchdog monitoring is running.

## Reference State (Gold Master)
- **Model**: ResNet-18 (Fusion Policy)
- **Scale**: 1.0x True Physics
- **Lateral Plateau**: 0.05m (Strict)
- **Smoothness Weight**: 15.0
- **Entropy Coef**: 0.01
- **Spawn Height**: 0.12m
- **Boundary Threshold**: 0.12m
- **Height Termination**: 0.02m
- **Envs**: 32
- **Sensors**: Enabled (Camera 224x224, tilted 30deg down)

## Active Todos (Queue)
1. [x] **16-01-REWARD-STABILITY**: Fix Greed Bug and Lazy Bureaucrat reward imbalances.
2. [x] **16-02-PROGRESS-REWARD**: Implement track-tangent projection for speed.
3. [x] **16-03-PHYSICS-STABILITY**: Confirm Gold Master orientation and drop logic.
4. [ ] **16-04-MARL-ARCHITECTURE**: Discuss refactoring RoadGraph to agent-indexed tensors.
