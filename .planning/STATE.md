# Project State: ARCPro RL v1.2-dev

## Current Milestone
**Milestone 1: Physical Fidelity & Policy Foundation** (Restored Baseline)

## Current Phase
**Phase 09: Training Loop Stabilization** (ACTIVE)

## Summary
The project is on the `dev` branch. Phase 09-01 and 09-02 are complete. Physics are stabilized for the 1500kg/8.0x robot with AWD mapping and mathematical centerline alignment. We are now initiating policy retraining (Phase 09-03).

## Recent Activity
- **Phase 09-01 Complete**: Enabled visuals, calibrated 8x physics, implemented raycast-snapped spawn, and aligned waypoints.
- **Phase 09-02 Refined**:
    - **1500kg Mass Correction**: Stabilized heavy chassis for realistic traction (~1400kg total).
    - **Action Space Refactor (2D Action Space)**: Identified that standard Isaac Lab `JointAction` terms expanded regex expressions into individual dimensions (6D total: 2 steer, 4 drive). 
    - **Grouped Actions Implementation**: Created `arcproLab/mdp/actions.py` with custom `GroupedJointAction` classes to collapse control into a stable 2D interface (1 Steering, 1 Throttle).
    - **Reset Logic**: Implemented 0.6m drift threshold and 50-step physics settling grace period.
- **Verification**: `check_action_space.py` confirms `Box(-inf, inf, (1, 2), float32)`. PPO retraining initiated with rising rewards.

## Known Issues & Solutions
1. **Excessive Wheel Spin**: Caused by 6D action space (uncoordinated wheels) and high throttle scale (40.0) vs low wheel mass (50kg). 
   - *Status*: Resolved by 2D Action Space grouping; agent now commands all drive wheels synchronously.
2. **Driving Backwards**: Policy initially explored negative rewards by reversing.
   - *Status*: Corrected by 2D action space (easier coordination) and reward monitoring.
3. **Rapid Terminations**: 0.6m lane limit is strict for a 3.6m wide robot at high speed.
   - *Status*: Monitoring; may require curriculum or slight limit relaxation if convergence stalls.

## Active Todos (Queue)
1. [x] **09-01-stabilize-loop**: Enable visuals, verify 8x scale, and initiate dry run.
2. [x] **09-02-physical-calibration**: Align spawn/waypoints, correct AWD mapping, and stabilize 1500kg physics.
3. [/] **09-03-policy-retraining**: (ACTIVE) Resume PPO training with 1 environment, 1500kg physics, and 2D action space.


## Blockers
None.
