# Phase 07: Revert to True Physics Mode - Summary (Grounding Task)

## Objective
Revert the simulation environment from "Giant Scale" (20x) back to "True Physics" (1.0x metric) for improved realism and transferability.

## Implementation Details
- **Robot Asset**: Switched to `F1Tenth_Metric.usd` (1.0x metric).
- **Track Scaling**: Applied `0.0825` scale factor to `no_graph_sim_cleaned.usd` to achieve ~3.5m lane widths.
- **Grounding**: Applied `-1.25m` Z-offset to the track prim to align the road surface with the world origin (Z=0).
- **Physics**: Removed high-damping/stiffness overrides (`setup_robot_stability` event) and reverted to stable (8, 4) solver iterations.
- **Telemetry**: Scaled `track_centerline.npy` by `0.0825` to ensure accurate lateral/heading error calculations in meters.
- **Camera**: Adjusted `tiled_camera` offset to `(0.28, 0.0, 0.16)` for the metric robot.

## Success Criteria Met
- [x] Simulation launches without errors at 1.0x metric scale.
- [x] Robot surface is grounded at Z=0.
- [x] Robot remains stable during drop tests and driving.
- [x] Telemetry accurately reports metric errors (verified with `verify_metric.py`).

## Impact
The simulation is now physically realistic and ready for high-fidelity training without the stability workarounds previously required for giant-scale assets.

## Challenges & Troubleshooting

### 1. Hidden Track Z-Offset
**Challenge**: The track asset (`no_graph_sim_cleaned.usd`) contained an internal Z-offset. When spawned at `(0,0,0)`, the actual road surface was elevated at approximately `Z ~15.15m` (unscaled).
**Discovery**: A 20m high-spawn test showed the robot falling only ~5m before hitting the track.
**Solution**: Applied a `-1.25m` offset to the track prim in `arcpro_env_cfg.py` (after `0.0825` scaling) to ground the road surface exactly at `Z=0`.

### 2. Metric Scaling Calculation
**Challenge**: Determining the precise scale factor to convert OpenStreetMap (OSM) units to meters.
**Discovery**: Measurement of the road lanes in the USD showed a width of `~42.4` units. Standard road lanes are `~3.5m`.
**Solution**: Calculated and applied a scale factor of `0.0825` (`3.5 / 42.4`) to the track asset.

### 3. Telemetry Scale Mismatch
**Challenge**: After scaling the environment to metric, the `TrackManager` reported massive lateral and heading errors.
**Discovery**: The waypoints in `track_centerline.npy` were still in raw OSM units (e.g., 300+ units long), while the robot was moving in meters (e.g., 30m long).
**Solution**: Scaled the `track_centerline.npy` file on disk by `0.0825` to align the waypoints with the metric simulation.

### 4. Robot Clipping & Physics Stability
**Challenge**: At 1.0x metric scale, the robot was prone to clipping through the road mesh or "jittering" on spawn.
**Discovery**: Standard PhysX iterations `(4, 1)` were insufficient for high-speed stability during the initial transition.
**Solution**: Maintained "Stable Metric" iterations of `(8, 4)` (Position, Velocity) and set the spawn height to `0.5m` to ensure a clean drop onto the grounded surface.

### 5. Broken Configuration Imports
**Challenge**: Verification scripts failed after repository cleanup.
**Discovery**: `verify_metric.py` was hard-coded to import from a temporary file `arcpro_metric_env_cfg.py` which was deleted during the merge.
**Solution**: Refactored `verify_metric.py` to import from the main `arcpro_env_cfg.py` and synchronized all configuration files.
