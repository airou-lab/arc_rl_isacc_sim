# Todo: USD Asset Flattening (The Scaling Hack)

**Priority**: Medium
**Status**: Queue
**Component**: Environment Configuration / USD Assets

## Problem
The source USD file for the track (`openStreetUSD/no_graph_sim_clean_1x.usda`) was authored at an 8x metric scale. In `arcproLab/arcpro_env_cfg.py`, a `scale=(0.125, 0.125, 0.125)` hack is applied to shrink it to the correct 1.0x metric scale matching the robot.

This is brittle. If a true 1.0x map is ever exported and used, it will be shrunk to 1/8th of its correct size, breaking the physics and collision geometry.

## Solution
1. Open the USD file in a tool or use Python USD API (`pxr.UsdGeom.XformCommonAPI(prim).SetScale((1.0, 1.0, 1.0))`) to permanently flatten the mesh geometry scale to 1.0x.
2. Save the flattened map as a new USD file (e.g., `no_graph_sim_clean_true_1x.usda`).
3. Update `arcpro_env_cfg.py` to remove the `0.125` scaling factor and load the new asset with `scale=(1.0, 1.0, 1.0)`.

## Verification
- Robot spawns correctly on the road without floating or falling through.
- Track waypoints (telemetry) perfectly align with the visual road lines.
- Lane width remains approximately 3.5m.