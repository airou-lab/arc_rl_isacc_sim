# Phase 11 Validation: Retraining & Intersection Logic

## Acceptance Criteria
- [ ] **PH11-01 (Telemetry Alignment):** Telemetry indices 0-11 correctly map to HPPP constants (Turn Token, Go Signal, Speed, Yaw Rate, Steer, Throttle, Brake, LatErr, HeadErr, Curvature, Distance).
- [ ] **PH11-02 (Physics Response):** The 1.0x metric robot responds correctly to CombinedDriveAction (throttle + brake).
- [ ] **PH11-03 (Convergence):** Training achieves Explained Variance > 0.6 and EpRew improvement on the 1.0x metric map.
- [ ] **PH11-04 (Boundary Integrity):** Interpolated road markers prevent "leakage" through sparse USD prims.

## Verification Methods
- **Unit Test:** `pytest tests/test_telemetry.py` (Verify index mapping).
- **Integration Test:** `python arcproLab/scripts/verify_policy.py --num_envs 1` (Visual check of boundary resets).
- **Training Metric:** Monitor `rollout/ep_len_mean` and `train/explained_variance` in TensorBoard.
