# Phase 11 Plan: Hierarchical Policy Integration

This plan details the migration of the `HierarchicalPathPlanningPolicy` (HPPP) from the reference stack into the `arcproLab` workspace at 1.0x metric scale.

## Goals
1.  **Zero-Rewrite Policy Swap**: Plug in the reference HPPP with no logic changes.
2.  **Telemetry Alignment**: Update `mdp/observations.py` to match the Protocol v2 contract.
3.  **Brake Support**: Implement `CombinedDriveAction` to handle the new [steer, throttle, brake] action space.
4.  **Waypoint Supervision**: Hook up `WaypointTrackingWrapper` for auxiliary supervised loss.

---

## 1. Environment Alignment (MDP Layer)
- [ ] **Task 11-02-obs-alignment**: Update `arcproLab/mdp/observations.py` to populate indices 0 (turn_token) and 1 (go_signal) with mocked values (STRAIGHT/GO).
- [ ] **Task 11-03-brake-action**: Create `CombinedDriveAction` in `arcproLab/mdp/actions.py` to process the [throttle, brake] fusion.
- [ ] **Task 11-04-env-config**: Update `arcproLab/arcpro_env_cfg.py` to use the new action term and rename observation keys to `image` and `vec`.

## 2. Policy Stack Integration
- [ ] **Task 11-05-submodule-init**: Add `arc_rl_isacc_policy` as a Git Submodule at `arcproLab/policy_stack`.
- [ ] **Task 11-06-train-script**: Update `arcproLab/scripts/train_policy.py` to use `sb3_contrib.RecurrentPPO` and the `HierarchicalPathPlanningPolicy` imported from the submodule.

## 3. Training & Validation
- [ ] **Task 11-07-smoke-test**: Run a 100-step training loop with 1 environment to verify no shape/index crashes.
- [ ] **Task 11-08-verify-math**: Use `verify_policy.py` to confirm that the `LatErr` math (fixed in Wave 0) is correctly feeding into the HPPP feature extractor.
- [ ] **Task 11-09-production-run**: Launch 5,000,000 step production training run.

---

## Verification Strategy
- **Perception Check**: Visualise markers in the GUI to confirm `LatErr` matches reality.
- **Control Check**: Verify that `brake=1.0` results in zero wheel velocity in the GUI telemetry.
- **Loss Check**: Monitor TensorBoard to ensure `waypoint_loss` (Auxiliary) is decreasing.
