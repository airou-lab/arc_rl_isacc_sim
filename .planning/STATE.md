# Project State: ARCPro RL v1.2-dev

## Current Milestone
**Milestone 1: Physical Fidelity & Policy Foundation** (Restored Baseline)

## Current Phase
**Phase 09: Training Loop Stabilization** (ACTIVE)

## Summary
The project has been reset to the **Ultimate Source of Truth** (Commit `519167d`). This baseline features the manually adjusted **8.0x metric robot scale**, which is the **intended and correct sizing** for the project to ensure proper interaction with the environment and physics stability. The world coordinates are fixed at `(-129.465, 46.927, 2.0)`. All subsequent work will be branched from this stable point on the new `dev` branch.

## Recent Activity
- **Phase Re-alignment**: Restructured planning files to reflect accurate phase progression (Phases 07 & 08 confirmed COMPLETE).
- **Source of Truth Restoration**: Hard reset `main` to `519167d`.
- **Branch Management**: Recreated `dev` branch from the new `main` and pushed all branches to origin.
- **Scale Confirmation**: Formally adopted 8.0x as the target robot scale.
- **Verification**: Confirmed 8.0x scale robot and fixed spawn coordinates in `arcpro_env_cfg.py`.

## Key Achievements
- **Stable Baseline**: Restored the "Monolith" state where the robot spawns and drives correctly in the `no_graph_sim.usd` scene.
- **Sizing Finalized**: Confirmed 8.0x scale is the project standard (Mandatory).
- **Manual Alignment**: Verified specific world coordinates for the robot spawn.
- **Documentation**: Updated planning docs to reflect the new baseline.

## Planned Tasks
- [ ] **Phase 09: Training Loop Stabilization** - Finalize reset logic and torque verification.
  - [ ] **Task 1**: Fix Raycast Mesh Detection (Fix the "FAILED to find road mesh" error).
  - [ ] **Task 2**: Torque Verification (Ensure AWD produces expected acceleration).
  - [ ] **Task 3**: Reward Balance (Verify reward ranges are not causing early termination).
- [ ] **Phase 10: Intersection Navigation** - Implement graph-based routing.

## Next Steps for /gsd:resume
1. **Enable Visuals**: In `arcproLab/arcpro_env_cfg.py`, set `enable_cameras: True`.
2. **Verify Camera Offset**: Confirm the `tiled_camera` offset `(2.24, 0.0, 1.28)` provides a clear front view for the 8.0x robot.
3. **Integrate Reset Logic**: Port the waypoint-based reset logic from `feat/waypoint-snapping` into the current `dev` branch.
4. **Torque Audit**: Run `verify_sim.sh` to confirm the 20kg 8x chassis has sufficient torque for stable lane-following.
5. **Finetune/Retrain**: If spawning and visuals are correct, initiate a training run using `train.sh` to adapt the policy to the 8.0x metric world.
