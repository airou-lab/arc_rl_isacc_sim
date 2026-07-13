# S01: Phase 16: MARL Transition Backlog

**Goal:** Complete the MARL transition backlog from the Phase 16 Gold Master checkpoint.
**Demo:** A fully vectorized multi-agent-ready environment with no left-drift, successfully mapped centerlines, and a user-approved HD vision policy.

## Must-Haves

- Training run approved.
- RoadGraph uses vectorized operations for N agents.
- Centerline KD-Tree correctly returns the right-turn intersection.
- Left-drift steering trim offset resolved or fully compensated.

## Proof Level

- This slice proves: High

## Integration Closure

No downstream milestones blocked.

## Verification

- Update metrics or print logs to handle multiple agent indexed variables safely.

## Tasks

- [x] **T01: Approve Training Run** `est:15m`
  Review the 32-env training run results with the user and get explicit approval to proceed.
  - Verify: User states 'approved' or 'yes'.

- [x] **T02: MARL Architecture Discussion (RoadGraph)** `est:1h`
  Discuss refactoring `RoadGraph` to use agent-indexed tensors (N-agents) instead of singular coordinates. Analyze codebase impact and plan refactor.
  - Files: `arcproLab/mdp/road_manager.py`
  - Verify: Research summary artifact saved with explicit refactoring plan.

- [ ] **T03: Centerline Regeneration Script** `est:1h`
  Write a KD-Tree script to properly map the right-turn intersection, addressing the 16-05 backlog item.
  - Files: `arcproLab/generate_track.py`
  - Verify: Script successfully executes and regenerates the centerline arrays with correct right-turn topology.

- [ ] **T04: Physics Left-Drift Bias Fix** `est:30m`
  Investigate the physical left-drift bias, adjust steering trim, and verify the vehicle drives straight under 0.0 actions.
  - Files: `arcproLab/arcpro_env_cfg.py`
  - Verify: Test environment shows F1Tenth driving straight without drifting left.

- [ ] **T05: Research SKRL AAC Integration** `est:30m`
  Research transitioning from SB3 to SKRL to natively support MARL and Asymmetric Actor-Critic (AAC).
  - Files: `tasks/T05-RESEARCH.md`
  - Verify: Research document accurately details SKRL integration path.

- [ ] **T06: Plan SKRL AAC Integration** `est:30m`
  Plan the refactor to migrate the training loop to SKRL and define the Actor/Critic models.
  - Files: `tasks/T05-PLAN.md`
  - Verify: Plan provides clear, actionable steps for Phase 16 execution.

## Files Likely Touched

- arcproLab/mdp/road_manager.py
- arcproLab/generate_track.py
- arcproLab/arcpro_env_cfg.py
