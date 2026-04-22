# Phase 11 Plan: Hierarchical Policy Integration

This plan details the migration of the `HierarchicalPathPlanningPolicy` (HPPP) from the reference stack into the `arcproLab` workspace at 1.0x metric scale.

## Goals
1.  **Zero-Rewrite Policy Swap**: Plug in the reference HPPP with no logic changes.
2.  **Telemetry Alignment**: Update `mdp/observations.py` to match the Protocol v2 contract.
3.  **Brake Support**: Implement `CombinedDriveAction` to handle the new [steer, throttle, brake] action space.
4.  **Waypoint Supervision**: Hook up `WaypointTrackingWrapper` for auxiliary supervised loss.
5.  **Robust Termination**: (NEW) Implement dense marker boundaries to prevent model "leakage" through divider lines.

---

## 1. Environment Alignment (MDP Layer)
- [x] **Task 11-02-obs-alignment**: Update `arcproLab/mdp/observations.py` to populate indices 0 (turn_token) and 1 (go_signal) with mocked values (STRAIGHT/GO).
- [x] **Task 11-03-brake-action**: Create `CombinedDriveAction` in `arcproLab/mdp/actions.py` to process the [throttle, brake] fusion.
- [x] **Task 11-04-env-config**: Update `arcproLab/arcpro_env_cfg.py` to use the new action term and rename observation keys to `image` and `vec`.

## 2. Policy Stack Integration
- [x] **Task 11-05-submodule-init**: Add `arc_rl_isacc_policy` as a Git Submodule at `arcproLab/policy_stack`.
- [x] **Task 11-06-train-script**: Update `arcproLab/scripts/train_policy.py` to use `sb3_contrib.RecurrentPPO` and the `HierarchicalPathPlanningPolicy` imported from the submodule.

## 3. Training & Validation
- [x] **Task 11-07-smoke-test**: Run a 100-step training loop with 1 environment to verify no shape/index crashes.
- [x] **Task 11-08-verify-math**: Use `verify_policy.py` to confirm that the `LatErr` math (fixed in Wave 0) is correctly feeding into the HPPP feature extractor.
- [x] **Task 11-09-production-run**: Launch 5,000,000 step production training run.

## 4. Robustness & Physics Tuning (Wave 1)
- [x] **Task 11-10-dense-wall**: Implement marker interpolation in `TrackManager` to create a solid boundary segment.
- [x] **Task 11-11-normal-spawn**: Implement Raycast-based ground-normal alignment in `events.py` for stable robot spawning.
- [x] **Task 11-12-centering-fix**: Recalibrate lane offset (0.30m) and spawn (X=-16.26) for optimal centering.
- [/] **Task 11-14-production-run-v6**: (RUNNING) Current production run with stabilized 5e-5 learning rate.

## 5. Intersection Logic (Wave 2)
- [ ] **Task 11-15-road-graph**: Implement `RoadGraph` class to handle multi-segment navigation.
- [ ] **Task 11-16-segment-discovery**: Add auto-discovery of road segments from the USD stage tree.

---

## Verification Strategy
- **Perception Check**: Visualise markers in the GUI to confirm `LatErr` matches reality.
- **Control Check**: Verify that `brake=1.0` results in zero wheel velocity in the GUI telemetry.
- **Boundary Check**: Use `test_straight_line.py` to confirm the robot resets at the lane edge.
