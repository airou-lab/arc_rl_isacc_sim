# Project State: ARCPro RL v1.2-dev

## Current Milestone
**Milestone 1: Physical Fidelity & Policy Foundation** (Restored Baseline)

## Current Phase
**Phase 09: Training Loop Stabilization** (ACTIVE)

## Summary
The project is on the `dev` branch. Phase 09-01 (Stabilization) is complete, with vision enabled and physics calibrated for the 8.0x robot scale. We are now in Phase 09-02, focusing on physical contact-based termination.

## Recent Activity
- **Phase 09-01 Complete**: Enabled visuals, calibrated 8x physics, implemented raycast-snapped spawn, and aligned waypoints.
- **Physics Stabilization (8x Scale)**: 
    - **Mass Correction**: Increased robot mass from 20kg to 1500kg for realistic traction and grip.
    - **Actuator Boost**: Increased steering and drive stiffness/damping (50k) to handle 1500kg mass.
    - **AWD Correction**: Identified and fixed 4-wheel inversion and joint mapping (Index 2-5).
    - **Spawn Alignment**: Zeroed initial lateral error (X=-130.03) to maximize lane wiggle room (0.6m limit).
- **Verification Complete**: Robot now drives forward stably in GUI for ~1000 steps without immediate physics-induced resets.

## Active Todos (Queue)
1. [x] **09-01-stabilize-loop**: Enable visuals, verify 8x scale, and initiate dry run.
2. [x] **09-02-physical-calibration**: Align spawn/waypoints, correct AWD mapping, and stabilize 1500kg physics.
3. [ ] **09-03-policy-retraining**: Resume PPO training with 2 environments and 1500kg physics.

## Blockers
None.
