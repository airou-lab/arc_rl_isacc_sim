# Phase 12 Validation: Autonomous Navigation (RoadGraph Triggers)

## Acceptance Criteria
- [ ] **NAV-TR-01 (Proximity Trigger):** `RoadGraph` correctly updates `turn_token` to non-zero when within 2.5m of a gate.
- [ ] **NAV-TR-02 (Token Accuracy):** The provided `turn_token` matches the `SignalTurnRelation` attribute (Left=-1, Straight=0, Right=1) from the USD.
- [ ] **NAV-TR-03 (Default Behavior):** The system defaults to `turn_token=0.0` (Straight) when no gate is in range.
- [ ] **NAV-TR-04 (Performance):** `RoadGraph.update()` uses vectorized `torch.cdist` and does not degrade FPS below 100 on 32 envs.

## Verification Methods
- **Unit Test:** `pytest tests/test_road_graph.py` (Verify proximity triggers).
- **GUI Test:** `python3 arcproLab/scripts/verify_policy.py --debug` (Observe Cyan/Magenta markers and console logs for turn tokens).
