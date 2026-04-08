# Codebase Concerns

**Analysis Date:** 2025-04-28

## Tech Debt

**[Hybrid Scale Complexity]:**
- Issue: Simulation operates at **8.0x robot scale**, policy at **1.0x metric scale**. Manual normalization (`* 0.125`) is required in `observations.py`.
- Files: `arcproLab/arcpro_env_cfg.py`, `arcproLab/mdp/observations.py`.
- Impact: Increased cognitive load and risk of scaling errors in telemetry.
- Mitigation: Centralized normalization logic in `observations.py`.

**[Incomplete Reset Logic]:**
- Issue: `reset_robot_to_lane` currently lands the robot from 1.0m above the hit point, which can cause bouncing and instability during training resets.
- Files: `arcproLab/mdp/events.py`.
- Impact: Physics instability at the start of episodes.
- Fix: Implement direct Z=0.05m snapping (Phase 09 Task).

## Known Bugs

**[Raycast Failure Fallback]:**
- Issue: When the raycast fails to find a "road" mesh, the robot falls back to a hardcoded Z=10.0 position.
- Files: `arcproLab/mdp/events.py`.
- Trigger: Gaps in road geometry or collision mesh issues in `no_graph_sim.usd`.
- Workaround: Increased retry count and broader mesh detection; needs final stabilization in Phase 09.

**[SB3 Import Compatibility]:**
- Issue: Isaac Lab updates changed the SB3 wrapper import paths.
- Files: `arcproLab/scripts/train_policy.py`.
- Status: Fixed (Now using `isaaclab_rl.sb3`).

## Security Considerations

**[Local Simulation]:**
- Risk: Low. All operations are local within the Isaac Sim container/environment.

## Performance Bottlenecks

**[Camera VRAM Overhead]:**
- Problem: Enabling cameras (`enable_cameras=True`) significantly increases VRAM usage.
- Files: `arcproLab/arcpro_env_cfg.py`.
- Cause: Rendering overhead for tiled cameras in Isaac Sim.
- Improvement path: Only enable cameras for visual verification or during specific vision-based training phases.

## Fragile Areas

**[Waypoint Alignment]:**
- Files: `arcproLab/mdp/track_centerline.npy`, `openStreetUSD/no_graph_sim.usd`.
- Why fragile: If the road mesh is translated or scaled in USD without regenerating waypoints, navigation will fail.
- Safety measure: Run `verify_spawn.py` after any USD modification.

**[Torque Calibration]:**
- Files: `arcproLab/arcpro_robot_cfg.py`.
- Why fragile: 20kg mass at 8x scale requires precise `effort_limit_sim` and `stiffness/damping` tuning to prevent oscillation or sluggishness.

## Scaling Limits

**[Multi-Agent Support]:**
- Current capacity: Single robot.
- Limit: Vectorized logic is mostly in place, but `TrackManager` and `events.py` need verification for large-scale multi-robot training (e.g., > 1024 environments).

## Missing Critical Features

**[Domain Randomization]:**
- Problem: Randomization of friction, mass, and actuator noise is missing.
- Blocks: Robust sim-to-real transfer and generalization.

## Test Coverage Gaps

**[Phase 09 Verification Loop]:**
- What's not tested: Automated verification of torque/acceleration performance.
- Files: `arcproLab/scripts/verify_metric.py`.
- Priority: High (Part of stabilization workflow).

**[Camera Blocking Checks]:**
- What's not tested: Whether the `tiled_camera` view is obstructed by the chassis at 8x scale.
- Files: `arcproLab/arcpro_env_cfg.py`.
- Priority: Medium (Phase 09 Task).

---

*Concerns audit: 2025-04-28*
