# Todo: Phase 09-02 Strict Termination & Lane Alignment

## Context
The 8x scale robot is significantly wider than a standard lane. Currently, the robot only resets when it is nearly entirely off the road. We need to enforce "any wheel" contact with the boundaries (white lines or yellow lines).

## Tasks
- [ ] **Task 1: Implement Strict Lane Enforcement**
  - Update `white_line_contact` to reset if any part of the 8x robot footprint (approx 2.4m wide) touches the road boundaries.
  - Calculated Threshold: `abs(LatErr) > 0.3m` (0.0375 normalized) assuming a 1.5m lane half-width.
  - *Status: Pending*

- [ ] **Task 2: Shift Spawn Point to Right Lane Center**
  - Update `arcpro_env_cfg.py` to spawn the robot at `LatErr = 0.75m` instead of the yellow centerline (`LatErr = 0.0m`).
  - *Status: Pending*

- [ ] **Task 3: Reduce Grace Period & Verify**
  - Lower the `settled` grace period from 100 steps to 25 steps once the spawn is aligned.
  - Run verification to confirm "any wheel" triggers reset.
  - *Status: Pending*
