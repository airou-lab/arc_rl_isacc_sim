# Project State: ARCPro RL v1.2-dev

## Current Phase
**Phase 9: Training Stabilization (8x Giant Scale)**

## Summary
The project is currently optimized for **8x Giant Scale** physics to ensure simulation stability. We have successfully implemented robust termination logic that detects "any wheel" contact with road boundaries (White/Yellow lines) and high-force chassis crashes. Rewards have been aligned with these strict boundaries to provide a clear training signal.

## Recent Activity
- **8x Giant Scale Transition**: Shifted simulation to 8.0x robot scale to resolve small-scale PhysX jitter issues.
- **Any Wheel Termination**: Implemented strict 0.3m (0.0375 normalized) lateral error threshold to trigger reset when any part of the 2.4m wide robot touches lane boundaries.
- **Robust Physics Masking**: Added a 5-step settling grace period and 5000N force threshold to ignore spawn jitter while catching actual crashes.
- **Reward Alignment**: Updated `lateral_error_reward` to match the 0.3m termination boundary, replacing the lenient 4.0m (0.5 normalized) legacy threshold.
- **Joint Mapping Correction**: Identified and fixed a mapping issue in `verify_policy.py` where drive commands were being sent to steering joints.

## Key Achievements
- **Stable 8x Simulation**: Physics operates reliably at giant scale without ragdolling or high-frequency jitter.
- **Precise Lane Boundaries**: Confirmed through visual verification that resets trigger accurately at the lane edges.
- **AWD Control Integration**: Successfully actuating all 4 wheels (Indices 2-5) for improved traction.

## Active Tasks (Phase 9)
- [ ] Monitor PPO training progress with the new strict rewards and termination.
- [ ] Evaluate 'Learning' performance: Check if the agent's mean episode length increases.
- [ ] Fine-tune AWD throttle coefficients if the 8x robot remains sluggish.

## Completed (Phase 09-02 - Robust Terminations)
- [x] Implement High-Force Chassis Crash Detection (5000N).
- [x] Implement Strict Lane Departure Termination (0.3m physical).
- [x] Synchronize Reward Thresholds with Termination logic.
- [x] Fix Spawning Height (0.5m) and Raycast Snapping.
- [x] Verify "Any Wheel" termination behavior visually.
