# Project State: ARCPro RL v1.2-dev

## Current Milestone
**Milestone 1: Physical Fidelity & Policy Foundation** (Restored Baseline)

## Current Phase
**Phase 09: Training Loop Stabilization** (ACTIVE)

## Summary
The project is on the `dev` branch. Phase 09-01 and 09-02 are complete. Physics are stabilized for the 1500kg/8.0x robot with AWD mapping and mathematical centerline alignment. We are now initiating policy retraining (Phase 09-03).

## Recent Activity
- **Phase 09-01 Complete**: Enabled visuals, calibrated 8x physics, implemented raycast-snapped spawn, and aligned waypoints.
- **Phase 09-02 Complete (Physical Calibration & Validation)**:
    - **Status**: VERIFIED. Agent successfully learning on heavy-chassis physics.
    - **1400kg Balanced Mass Distribution**: Corrected physical imbalance using a custom spawner (`arcproLab/mdp/spawner.py`).
    - **Actuator Gain Overhaul**: Stabilized 1200kg chassis with 500,000 stiffness gains.
    - **Normalization Breakthrough**: Identified that verification scripts must load `vec_normalize.pkl` to match the policy's input space. Implemented periodic normalization saves in `train_policy.py`.
- **Phase 09-03 Active**: PPO retraining showing strong convergence. `ep_rew_mean` surged from -3.09 to +24.9 at 40k steps.

## Known Issues & Solutions
1. **Normalization Mismatch**: Resolved by implementing `SaveVecNormalizeCallback` to save stats every 10k steps.
2. **Visual Misalignment at End-of-Track**: Confirmed Waypoint 999 does not align with visual mesh; verification focused on Waypoint 0.
3. **Excessive Wheel Spin**: Resolved by 2D Action Space sync and 5,000 actuator damping.

## Active Todos (Queue)
1. [x] **09-01-stabilize-loop**: Enable visuals, verify 8x scale, and initiate dry run.
2. [x] **09-02-physical-calibration**: (COMPLETE) Verified stable physics and learning convergence.
3. [/] **09-03-policy-retraining**: (ACTIVE) PPO training progressing healthy (~17 FPS).


## Blockers
None.
