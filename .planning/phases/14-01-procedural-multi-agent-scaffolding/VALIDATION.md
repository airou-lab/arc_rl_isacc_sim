# Phase 14 Validation: Multi-Agent Environment Refactor

## Acceptance Criteria
- [ ] **MARL-SPAWN-01 (Procedural Spawning):** The environment successfully spawns N robots (N=4) using a loop in `ARCProSceneCfg`.
- [ ] **MARL-V2V-02 (Queue Logic):** The singleton V2V Manager correctly identifies Gate triggers and maintains an FCFS queue.
- [ ] **MARL-OBS-03 (Token Delivery):** Robots receive their `Go/Wait` and `Queue Position` signals in the policy observation vector.
- [ ] **MARL-SAFE-04 (Global Reset):** Any robot-robot collision triggers an immediate reset of all agents in that environment instance.

## Verification Methods
- **Unit Test:** `pytest tests/test_v2v_manager.py` (Verify FCFS logic).
- **Integration Test:** `python3 arcproLab/scripts/verify_marl_spawning.py` (Visual confirmation of 4 robots).
- **Safety Test:** Manually drive one robot out of turn and confirm global environment reset.
