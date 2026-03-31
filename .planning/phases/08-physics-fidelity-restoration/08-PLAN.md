# Phase 08 Plan: F1Tenth Physics Fidelity Restoration

## Goal
Restore the high-fidelity physics parameters (damping, stiffness, friction) from the original F1Tenth assets to the current 1.0x metric simulation. This ensures realistic handling and stability while maintaining the true-to-scale environment.

## Context
During Phase 7 (True Physics Revert), several "highly tuned" physics parameters were removed as they were considered workarounds for the previous "Giant Scale" (20x) simulation. The user has requested to restore these original high-fidelity settings to the 1.0x metric model.

## Key Parameters to Restore
- **Friction**: Static Friction = 2.0, Dynamic Friction = 1.5 (previously applied to `/Full_Car/HighFrictionMaterial`).
- **Steering Damping**: 100.0 (currently 10.0 in `arcpro_robot_cfg.py`).
- **Steering Stiffness**: 1000.0 (currently 1000.0).
- **Drive Damping**: 10.0 (currently 10.0).
- **Drive Stiffness**: 0.0 (currently 0.0).
- **Contact Offset**: 0.02 (2cm) for collision meshes.

## Tasks
1. [ ] **Friction Restoration**: Apply high-fidelity friction materials to the wheels in `F1Tenth_Metric.usd`.
2. [ ] **Physics Parameter Alignment**: Update `arcpro_robot_cfg.py` with restored damping and stiffness values.
3. [ ] **Contact Offset Adjustment**: Apply 2cm contact offset to robot collision meshes for better stability.
4. [ ] **Verification**: Run `verify_sim_metric.sh` and perform human-in-the-loop stability tests (drop test, driving).

## Success Criteria
- [ ] Robot exhibits high-fidelity handling (high friction).
- [ ] Steering is stable and responsive (high damping).
- [ ] Simulation remains stable at 1.0x metric scale with (8, 4) solver iterations.
- [ ] No clipping or jitter during high-speed maneuvers.
