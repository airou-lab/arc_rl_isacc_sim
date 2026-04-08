# Project State: ARCPro RL v1.2-dev

## Current Milestone
**Milestone 1: Physical Fidelity & Policy Foundation** (Restored Baseline)

## Current Phase
**Phase 09: Training Loop Stabilization** (ACTIVE)

## Summary
The project is on the `dev` branch. Phase 09-01 (Stabilization) is complete, with vision enabled and physics calibrated for the 8.0x robot scale. We are now in Phase 09-02, focusing on physical contact-based termination.

## Recent Activity
- **Phase 09-01 Complete**: Enabled visuals, calibrated 8x physics, implemented raycast-snapped spawn, and aligned waypoints.
- **Merge to Main**: Integrated Phase 09-01 into main branch.
- **Phase 09-02 Initialized**: Implemented Roadmark Contact Termination using `ContactSensorCfg` and `white_line_contact`.

## Active Todos (Queue)
1. [x] **09-01-stabilize-loop**: Enable visuals, verify 8x scale, and initiate dry run.
2. [ ] **09-02-contact-termination**: Migrate from distance-based to physical contact-based lane resets.

## Blockers
None.
