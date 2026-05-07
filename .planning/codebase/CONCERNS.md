# Codebase Concerns

**Analysis Date:** 2024-11-20

## Tech Debt

**Marker Initialization:**
- Issue: `TrackManager` collects marker points from USD stage by scanning names. This is slow and fragile if naming conventions in the USD change.
- Files: `arcproLab/mdp/track_manager.py`
- Impact: Slow startup times on new tracks.
- Fix approach: Transition fully to the cached `.npz` system or use a more robust USD query API.

**CLI-driven Debugging:**
- Issue: Debugging visuals are toggled by checking `sys.argv` directly in `TrackManager.__init__`.
- Files: `arcproLab/mdp/track_manager.py`
- Impact: Unexpected behavior if `--debug` is used for other purposes or if `TrackManager` is used in a context without CLI args.
- Fix approach: Use a proper configuration object or environment variable.

## Known Bugs

**None reported:**
- Currently, the focus is on migration and scale verification (1.0x metric scale).

## Security Considerations

**Environment Variables:**
- Risk: None detected (local simulation).
- Files: N/A
- Current mitigation: N/A

## Performance Bottlenecks

**USD Point Collection:**
- Problem: Scanning the USD stage for "yellow_line", "white_line", etc., is slow.
- Files: `arcproLab/mdp/track_manager.py`
- Cause: Iterating over many USD prims.
- Improvement path: Ensure the `.npz` cache is always up to date and shared between environments.

## Fragile Areas

**USD Path Dependencies:**
- Files: `arcproLab/arcpro_env_cfg.py`, `arcproLab/mdp/track_manager.py`
- Why fragile: Hardcoded relative paths to USD files and centerline data.
- Safe modification: Use absolute paths or an environment variable for the workspace root.

**Spawn/Reset Logic:**
- Files: `arcproLab/mdp/spawner.py`, `arcproLab/scripts/verify_spawn.py`
- Why fragile: Resetting state in Isaac Sim can occasionally lead to physics instability if joints aren't settled correctly.

## Scaling Limits

**Simulation Parallelization:**
- Current capacity: `num_envs` is usually set to 16-64.
- Limit: Limited by GPU VRAM and CPU core count for physics.
- Scaling path: Distribute training across multiple GPUs if needed.

## Dependencies at Risk

**Isaac Lab/Sim:**
- Risk: Rapidly evolving API; updates to Isaac Sim often break backward compatibility.
- Impact: Env configs might break on newer versions.

## Missing Critical Features

**RoadGraph Connectivity:**
- Problem: While `RoadGraph` exists, full autonomous navigation through arbitrary intersections is still under development.
- Blocks: Multi-segment pathfinding.

## Test Coverage Gaps

**MDP Logic:**
- What's not tested: `rewards.py`, `observations.py` lack dedicated unit tests.
- Files: `arcproLab/mdp/rewards.py`, `arcproLab/mdp/observations.py`
- Risk: Reward bugs (e.g., sign errors) can go unnoticed and derail training.
- Priority: High.

---

*Concerns audit: 2024-11-20*
