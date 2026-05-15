# RL Simulation Stability (Legacy Logic)

> **STATUS:** PORTING TO ISAAC LAB. This document details the physics stability overrides discovered during Phase 2.

## Porting Targets for Isaac Lab
The following stability parameters from the legacy Direct API must be implemented in the new `ArticulationCfg` and `SimCfg`:

1.  **Damping & Stiffness:** 50.0 damping / 1000.0 stiffness across all 34 joints.
2.  **Physics Solver:** Position Iterations (64), Velocity Iterations (32).
3.  **Mass Force-Injection:** 3.0kg chassis, 0.15kg wheels.

## Asset Verification
- **Primary Asset:** `src/examples/ARCPro_RL/arc_rl_isacc_sim/f1tenth_trainer/assets/F1Tenth_Metric_Baked.usd`
- **Correction Applied:** Scaled `physics:localPos` attributes by 0.01 to fix the 100x joint offset error.
