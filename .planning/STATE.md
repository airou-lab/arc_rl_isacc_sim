# Project State: ARCPro RL v2.8-marl-stabilized

**RESUME HERE**
- **Milestone**: Milestone 3
- **Phase**: Phase 17 (Smart Intersection Infrastructure)
- **Next Todo**: Transition to intersection control logic and dynamic signaling.

---

## Current Milestone
**Milestone 3: HD Perception & Production Hardening** (ACTIVE)

## Current Phase
**Phase 16: MARL Transition** (STABILIZED)

## Summary
Phase 16 has been stabilized on the `main` branch. The architecture now supports high-speed lapping with true 1.0x metric physics and vectorized navigation management. Critical fixes for robot orientation, spawn stability (anti-wheelie), and precision gate visualization (0.05m) have been implemented and verified. The environment is optimized for high-FPS training on consumer-grade hardware (RTX 3060).

## Recent Activity (Phase 16 Stabilization)
- **Verified Scaling**: Synchronized track and robot scaling to 1.0x native metric consistency.
- **Fixed Orientation**: Corrected spawn heading to North-facing (1.5708 rad) for alignment with road graph.
- **Improved Spawn Stability**: Lowered spawn height to 4cm and disabled initial randomized velocity to eliminate "wheelie" flips.
- **Physics Tuning**: Applied 1.0 throttle damping and 0.01 armature to stabilize high-torque drive joints.
- **Termination Threshold**: Active `roadmark_contact` termination with a strict 0.15m threshold.
- **Visual Improvements**: Precision 0.05m spheres at gates and boundaries (subsampled for clarity).
- **Performance Optimization**: Tuned for 8-16 environments to ensure VRAM/FPS safety on RTX 3060 with HD cameras (224x224).

## Reference State (v2.8)
- **Model**: ResNet-18 (Fusion Policy)
- **Scale**: 1.0x True Physics (0.125 internal)
- **Orientation**: 1.5708 rad (North)
- **Spawn Height**: 0.04m
- **Damping**: 1.0 (Throttle) / 2.0 (Steering)
- **Envs**: 8-16
- **Sensors**: Enabled (Camera 224x224)

## Active Todos (Queue)
1. [x] **16-01-PHYSICS**: Fix drive polarity and clamping for high-speed stability.
2. [x] **16-02-CACHE**: Regenerate track boundary cache to fix 2.5m displacement.
3. [x] **16-03-MARL-STABILITY**: Fix orientation, scaling, and gate visuals.
4. [ ] **17-smart-intersection**: Implement intersection controllers (V2I/V2V). (NEXT)
