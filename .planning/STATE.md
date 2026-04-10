# Project State: ARCPro RL v1.2-dev

## Current Milestone
**Milestone 1: Physical Fidelity & Policy Foundation** (Restored Baseline)

## Current Phase
**Phase 09: Training Loop Stabilization** (ACTIVE)

## Summary
The project is on the `dev` branch. Phase 09-01 and 09-02 are complete. Physics are stabilized for the 1500kg/8.0x robot with AWD mapping and mathematical centerline alignment. We are now initiating policy retraining (Phase 09-03).

## Recent Activity
- **Phase 09-01 Complete**: Enabled visuals, calibrated 8x physics, implemented raycast-snapped spawn, and aligned waypoints.
- **Phase 09-02 (Physical Calibration - Verified)**:
    - **Status**: Physically Stable. Pending Policy Retraining confirmation.
    - **1400kg Balanced Mass Distribution**: Corrected physical imbalance by implementing a custom spawner (`arcproLab/mdp/spawner.py`) to apply realistic mass: Chassis (1200kg), Wheels (75kg), Knuckles (10kg).
    - **Actuator Gain Overhaul**: Massively increased steering and throttle gains (Stiffness: 500,000, Damping: 10,000) to stabilize the 1200kg heavy chassis.
    - **Safe Spawn Height**: Set spawn height to 1.0m (`arcproLab/mdp/events.py`) to prevent 3.6m wide wheels from clipping into the ground plane.
    - **Action Space Refactor (2D Action Space)**: Implemented `GroupedJointAction` classes to collapse control into a stable 2D interface (1 Steering, 1 Throttle).
    - **Verification**: `check_bias.py` confirms drift is reduced to <2 degrees per 100 steps. 
- **Phase 09-03 Initiated**: PPO retraining started with stabilized 1400kg physics and 2D action space.

## Known Issues & Solutions
1. **Convergence Uncertainty**: While physics are stable, it is not yet confirmed if the agent can effectively learn complex maneuvers with the 1200kg mass.
2. **Excessive Wheel Spin**: Resolved by 2D Action Space grouping and increased actuator damping.
3. **Physical Bias/Drift**: Resolved by balancing link masses and increasing actuator gains.

## Active Todos (Queue)
1. [x] **09-01-stabilize-loop**: Enable visuals, verify 8x scale, and initiate dry run.
2. [/] **09-02-physical-calibration**: (VERIFIED PHYSICAL) Pending confirmation of policy learning on heavy-chassis physics.
3. [/] **09-03-policy-retraining**: (ACTIVE) Resume PPO training with 1 environment, 1400kg balanced physics, and 2D action space.


## Blockers
None.
