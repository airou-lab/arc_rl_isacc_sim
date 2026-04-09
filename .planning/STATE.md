# Project State: ARCPro RL v1.2-dev

## Current Milestone
**Milestone 1: Physical Fidelity & Policy Foundation** (Restored Baseline)

## Current Phase
**Phase 09: Training Loop Stabilization** (ACTIVE)

## Summary
The project is on the `dev` branch. Phase 09-01 and 09-02 are complete. Physics are stabilized for the 1500kg/8.0x robot with AWD mapping and mathematical centerline alignment. We are now initiating policy retraining (Phase 09-03).

## Recent Activity
- **Phase 09-01 Complete**: Enabled visuals, calibrated 8x physics, implemented raycast-snapped spawn, and aligned waypoints.
- **Phase 09-02 Complete**:
    - **1500kg Mass Correction**: Stabilized heavy chassis for realistic traction.
    - **AWD Joint Mapping**: Corrected index 2-5 mapping and forward direction signs.
    - **Reset Logic**: Implemented 0.6m drift threshold and 20-step settling grace period.
    - **Environment Consolidation**: Reduced to 1 environment to bypass world-coordinate drift in secondary envs.
- **Verification**: Robot drives stably in GUI for ~1000 steps; math and visuals are perfectly aligned at X=-130.03.

## Active Todos (Queue)
1. [x] **09-01-stabilize-loop**: Enable visuals, verify 8x scale, and initiate dry run.
2. [x] **09-02-physical-calibration**: Align spawn/waypoints, correct AWD mapping, and stabilize 1500kg physics.
3. [ ] **09-03-policy-retraining**: Resume PPO training with 1 environment and 1500kg physics.


## Blockers
None.
