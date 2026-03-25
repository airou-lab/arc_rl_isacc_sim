# ARCPro RL: Isaac Sim Migration

## Current Status
- **Phase**: 04 - Robot Refinement & Verification (In Progress)
- **Baseline**: `no_graph_sim_cleaned.usd` at 1:1 proportions.
- **Active Challenge**: Physics stability at "Giant Scale" (20.0x robot).

## Recent Discoveries
- **Map Scaling**: The reference `no_graph_sim.usd` map uses a 42-meter-wide robot as its baseline, requiring a ~170x scale-up for a standard F1Tenth model to fit lanes.
- **Physics Solver Limits**: Scaling articulations to 170x causes immediate PhysX engine crashes. Stabilized at 20.0x scale using high-inertia tuning.
- **Sensor Stability**: Switched from `TiledCamera` to standard `CameraCfg` to prevent initialization hangs during high-scale environment creation.

## Active Constraints
- **Robot Scale**: 20.0x (Experimental for lane alignment).
- **Spawn Height**: 20.0m (Safety drop to clear USD mesh overlaps).
- **Physics**: 200Hz (dt=0.005), CCD disabled for debug.
