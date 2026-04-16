# Codebase Concerns

**Analysis Date:** 2025-05-15

## Tech Debt

**[Scale Ratio Maintenance]:**
- Issue: The 1x metric scale relies on a fixed ratio between the 1.0x robot and 0.125x track. This is manually configured in `arcpro_env_cfg.py`.
- Files: `arcproLab/arcpro_env_cfg.py`.
- Impact: Modifying the track USD without adjusting the scale factor will break physics and waypoint alignment.
- Fix approach: Formalize the scale factor in a central config or auto-calculate based on USD metadata if available.

**[Stochastic Reset Delay]:**
- Issue: `reset_robot_to_fixed_spawn` and other events have 0 grace period for physics settling, which may cause jitter at the start of episodes.
- Files: `arcproLab/mdp/events.py`.
- Impact: Initial observations might be noisy due to physics solver stabilization.

## Known Bugs

**[Raycast Snapping Jitter]:**
- Issue: In some sections of the track, the raycast may hit internal geometry, causing the robot to spawn slightly above or below the road surface.
- Files: `arcproLab/mdp/events.py`.
- Trigger: Complex mesh intersections in `no_graph_sim_clean_1x.usda`.
- Workaround: Snapping logic includes a small Z offset (+0.05m).

## Security Considerations

**[Headless Execution]:**
- Risk: Low. The environment is designed for local or private server execution.

## Performance Bottlenecks

**[Camera Rendering Overhead]:**
- Problem: High-frequency camera rendering (20Hz) across many environments significantly impacts FPS and VRAM.
- Files: `arcproLab/arcpro_env_cfg.py`.
- Cause: Isaac Sim rendering pipeline overhead for tiled cameras.
- Improvement path: Optimize camera resolution (currently 160x90) or use synthetic data generation for training where possible.

## Fragile Areas

**[Waypoint-USD Sync]:**
- Files: `arcproLab/mdp/track_centerline_1x.npy`, `openStreetUSD/no_graph_sim_clean_1x.usda`.
- Why fragile: Waypoints are absolute coordinates. Moving the track in the USD world without regenerating waypoints breaks navigation.
- Safe modification: Use `verify_spawn.py` to check alignment after any USD change.

**[Torque/Effort Calibration]:**
- Files: `arcproLab/arcpro_robot_cfg.py`.
- Why fragile: 1x scale physics is sensitive to small changes in effort limits and damping.
- Test coverage: `verify_metric.py` provides joint velocity audits.

## Scaling Limits

**[Environment Count]:**
- Current capacity: Tested up to 32 environments with cameras.
- Limit: Limited by VRAM for rendering; training without cameras can scale significantly higher.

## Missing Critical Features

**[Domain Randomization]:**
- Problem: Lack of friction and mass randomization.
- Blocks: Generalization for sim-to-real transfer.

## Test Coverage Gaps

**[Corner Case Navigation]:**
- What's not tested: Navigation behavior on steep inclines or sharp hairpins if added to the track.
- Files: `arcproLab/mdp/track_manager.py`.
- Priority: Medium.

**[Collision Penalty Tuning]:**
- What's not tested: The effectiveness of the `roadmark_contact` termination in discouraging cutting corners.
- Priority: High.

---

*Concerns audit: 2025-05-15*
