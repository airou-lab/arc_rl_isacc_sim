# Phase 11 Plan: Hierarchical Policy Integration

This plan details the migration of the `HierarchicalPathPlanningPolicy` (HPPP) from the reference stack into the `arcproLab` workspace at 1.0x metric scale.

## Goals
1.  **Zero-Rewrite Policy Swap**: Plug in the reference HPPP with no logic changes.
2.  **Telemetry Alignment**: Update `mdp/observations.py` to match the Protocol v2 contract.
3.  **Brake Support**: Implement `CombinedDriveAction` to handle the new [steer, throttle, brake] action space.
4.  **Robust Termination**: (NEW) Implement dense marker boundaries to prevent model "leakage" through divider lines.
5.  **Intersection Navigation**: (NEW Wave 2) Implement semantic filtering and direction-aware resets to allow crossing intersection stop lines.

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
- [/] **Task 11-14-production-run-v8**: (RUNNING) Current production run from 3.0M checkpoint.

## 5. Intersection Navigation (Wave 2)
- [x] **Task 11-15-semantic-filtering**: (COMPLETED) Update `TrackManager.py` to distinguish between `lane_boundary` and `stop_line` based on USD prim attributes.
- [x] **Task 11-16-direction-aware-resets**: (COMPLETED) Modify `terminations.py` to ignore stop-line resets when the robot has forward velocity/intent.
- [x] **Task 11-17-road-graph**: (COMPLETED) Implement `RoadGraph` class to handle multi-segment navigation and segment auto-discovery.

---

## Verification Strategy
- **Perception Check**: Visualise markers in the GUI to confirm `LatErr` matches reality.
- **Intersection Check**: Confirm the robot can cross a horizontal white line at an intersection without resetting.
- **Boundary Check**: Use `test_straight_line.py` to confirm the robot still resets at the side lane edges.
