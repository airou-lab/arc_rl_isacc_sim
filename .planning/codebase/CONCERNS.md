# Codebase Concerns

**Analysis Date:** 2025-03-31

## Tech Debt

**Incomplete Telemetry Protocol:**
- Issue: The `get_telemetry_vector` function only populates 7 of the 12 indices required by the legacy ARCPro policy protocol. Indices for acceleration, target distance, and specific status flags are missing.
- Files: `arcproLab/mdp/observations.py`
- Impact: Policies expecting the full 12-float vector will receive zeros for critical inputs, likely leading to poor performance or divergence.
- Fix approach: Implement calculation logic for missing indices (acceleration, target distance, etc.) or update the policy to accept a 7-float vector.

**Broken Reward & Termination Dependencies:**
- Issue: `line_penalty` reward depends on a `white_line_contact` termination term that is not defined in `TerminationCfg`. Furthermore, `white_line_contact` depends on a `contact_forces` sensor that is not defined in `ARCProSceneCfg`.
- Files: `arcproLab/mdp/rewards.py`, `arcproLab/mdp/terminations.py`, `arcproLab/arcpro_env_cfg.py`
- Impact: Runtime errors (KeyError) when calculating rewards or terminations during training.
- Fix approach: Add `ContactSensorCfg` to `ARCProSceneCfg` and register `white_line_contact` in `TerminationCfg`.

**Hardcoded Policy Assumptions:**
- Issue: `PolicyWrapper` contains hardcoded values for target speed (2.0 m/s) and wheel radius (0.05 m) which should ideally be synchronized with the environment configuration.
- Files: `arcproLab/mdp/policy_wrapper.py`
- Impact: Inconsistency between the policy's internal model of the robot and the actual simulation physics, leading to sub-optimal control.
- Fix approach: Pass these parameters from the configuration class to the wrapper.

## Known Bugs

**Height Termination Trigger at Spawn:**
- Issue: `height_termination` is set to trigger if `height > 0.3m`, but the robot is spawned (dropped) at `0.5m` in `arcpro_env_cfg.py`.
- Files: `arcproLab/mdp/terminations.py`, `arcproLab/arcpro_env_cfg.py`
- Trigger: Enabling `height_termination` in `TerminationCfg` will cause immediate episode termination upon reset.
- Workaround: Keep `height_termination` commented out (current state) or increase the threshold to `> 1.0m`.

**Waypoint Alignment Mismatch:**
- Issue: `TrackManager` shifts all sampled waypoints so the first point is at `(0,0)`. However, the simulation spawns the track and robot at world `(0,0)` without verifying that the track's internal road surface actually starts at `(0,0)`.
- Files: `arcproLab/mdp/track_manager.py`, `arcproLab/arcpro_env_cfg.py`
- Symptoms: Robot spawns at `(0,0)` but the telemetry (lateral error) indicates it is far from the track, or vice versa.
- Fix approach: Implement a "spawn alignment" check in `TrackManager` or `events.py` to ensure the robot is teleported to the actual first waypoint in world coordinates.

## Security Considerations

**None Detected:**
- Risk: Standard local simulation environment. No network-facing services or sensitive data detected.

## Performance Bottlenecks

**High-Density Waypoint Queries:**
- Problem: `TrackManager.get_closest_waypoint_data` performs a full brute-force search across all waypoints for every environment at every step.
- Files: `arcproLab/mdp/track_manager.py`
- Cause: `torch.argmin(dist_sq, dim=-1)` over potentially thousands of waypoints.
- Improvement path: Implement a spatial partitioning scheme (e.g., KD-Tree) or localized search around the previous closest waypoint index.

## Fragile Areas

**Metric Scaling (1.0x Mode):**
- Files: `arcproLab/arcpro_env_cfg.py`
- Why fragile: The track uses a "magic number" scale of `0.0825` to achieve "1.0x metric scale". If the source USD is modified or replaced, this scale factor will likely break. 
- Safe modification: Document the source of the `0.0825` factor (e.g., if it was converted from inches or feet).
- Test coverage: No automated test currently verifies the physical dimensions of the spawned track vs. expected metric units.

## Scaling Limits

**Vectorized Waypoint Search:**
- Current capacity: Efficient up to ~1000 envs / 5000 waypoints.
- Limit: GPU memory and compute time for the $O(N \cdot M)$ distance matrix will bottleneck at very high environment counts or extremely long tracks.
- Scaling path: Move to a $O(N)$ local search or spatial index.

## Dependencies at Risk

**Isaac Lab Compatibility:**
- Risk: The project relies on specific Isaac Lab manager patterns which are evolving rapidly.
- Impact: Future updates to Isaac Lab may break the custom observation/reward managers.

## Missing Critical Features

**Lane-Aligned Spawning:**
- Problem: The robot is always reset to `(0,0,0.5)`, which may not be on the track or in the correct orientation for training.
- Blocks: Effective training on complex tracks where the start line is not at the origin.
- Files: `arcproLab/mdp/events.py` (Currently a placeholder).

## Test Coverage Gaps

**Simulation-Telemetry Alignment:**
- What's not tested: No test verifies that a robot at a specific world coordinate receives the correct lateral/heading error for the actual USD track being used.
- Files: `tests/test_track_manager.py` (Only uses synthetic waypoints).
- Risk: Silent errors where the robot learns to follow "ghost" waypoints that don't match the visual track.
- Priority: High

---

*Concerns audit: 2025-03-31*
