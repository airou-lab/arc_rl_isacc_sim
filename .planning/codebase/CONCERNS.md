# Codebase Concerns

**Analysis Date:** 2024-10-24

## Tech Debt

**[Trash/Tools]:**
- Issue: A large number of scripts in `trash/tools/` seem critical for asset preparation but are stored in a "trash" directory.
- Files: `trash/tools/*.py`
- Impact: Confusing for new developers; hard to maintain.
- Fix approach: Move critical tools (like `finalize_track_gsd.py`) to a `tools/` or `scripts/utils/` directory and document them.

**[Scale Workarounds]:**
- Issue: The environment uses a magic scale `0.0825` for the track in `arcpro_env_cfg.py`.
- Files: `arcproLab/arcpro_env_cfg.py`
- Impact: Hard to reason about physical units if the base assets are not already at 1:1 scale.
- Fix approach: Rescale the master USD asset (`no_graph_sim_final.usd`) to 1:1 metric once and remove the scale factor from the code.

## Known Bugs

**[OSM Invisible Barriers]:**
- Issue: Tile junctions in OSM-generated tracks sometimes have "invisible barriers" or collisions.
- Files: `openStreetUSD/` assets.
- Trigger: Robot crossing a tile boundary.
- Workaround: Using `no_graph_sim_final.usd` which is flattened and cleaned.

## Security Considerations

**[Local Simulation]:**
- Risk: None identified; simulation runs locally with standard dependencies.

## Performance Bottlenecks

**[Solvers]:**
- Problem: High precision solvers (32 pos, 16 vel iterations) are enabled for the robot.
- Files: `arcproLab/arcpro_robot_cfg.py`.
- Cause: Required for stable F1Tenth physics (sim-to-real fidelity).
- Improvement path: Optimize collision geometry (using primitive hulls) to reduce simulation load.

## Fragile Areas

**[Waypoint Precision]:**
- Files: `arcproLab/mdp/track_centerline.npy`.
- Why fragile: If the track USD is updated but the centerline is not re-generated, navigation will fail or behave incorrectly.
- Test coverage: `tests/test_track_manager.py` checks the logic but not the actual data validity.

## Scaling Limits

**[Single Track]:**
- Current capacity: Single track lane following.
- Limit: Complex intersection navigation (graph-based).
- Scaling path: Move to Milestone 2 (v2.0) with graph navigation logic (`policy_stack/agent/intersection_graph.py`).

## Dependencies at Risk

**[sb3-contrib RNN support]:**
- Risk: `sb3-contrib`'s RecurrentPPO has specific conventions (e.g., RNN state tuple format) that have caused AttributeError in the past.
- Impact: Training instability or crashes after library updates.
- Migration plan: Keep version pins or consider custom LSTM implementation in the policy.

## Missing Critical Features

**[Metric Spawning]:**
- Problem: Spawning is currently tied to static waypoints.
- Blocks: Domain randomization for more robust policies.

## Test Coverage Gaps

**[MDP Rewards]:**
- What's not tested: Reward functions are complex (Gaussian weighted) and untested in isolation.
- Files: `arcproLab/mdp/rewards.py`.
- Risk: Unintended reward shaping could lead to poor policy performance.
- Priority: Medium.

---

*Concerns audit: 2024-10-24*
