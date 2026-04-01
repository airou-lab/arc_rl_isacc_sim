# Asset Verification Summary: Phase 05-01

## Status
**STALE** (Investigated, but "Invisible Barriers" remain a concern for revisit)

## Asset Selection (Draft)
- **Definitive Track**: `openStreetUSD/original_production.usd` (Created via Bake & Harden)
- **Definitive Robot**: `arcproLab/assets/robot/F1Tenth_Metric.usd` (Updated with 3.5kg Mass)

## Findings: The Collision & Barrier Issue
Our investigation into the "Original USD" (OSM Export) revealed a fundamental conflict between visual geometry and PhysX requirements.

### Root Cause Analysis & Resolution (Incomplete)
1.  **Mesh Thinness & Clipping**: The raw meshes had zero thickness, causing "tunneling."
    - **Fix**: Flattened all 527 mesh references and applied `convexDecomposition` globally. This made the floor physically solid.
2.  **Zero Traction**: The original robot had `0kg` mass and the road had no friction material.
    - **Fix**: Applied `3.5kg` mass to the car and bound a `Friction 2.0` physics material to all track meshes.
3.  **Invisible Barriers (The Final Boss)**: The map is composed of hundreds of non-manifold, tilted tiles. The physics hulls for these tiles do not "stitch" together perfectly.
    - **Status**: Small wheels (5cm radius) hit "micro-cliffs" (1-2cm jumps) at tile boundaries, causing the robot to stop (Vel=0) despite spinning wheels. This requires further research or a move to a single unified collision mesh.

### Technical Configuration
- **Track**: `original_production.usd`
- **Track init_state (pos)**: `(0.0, 0.0, 0.0)`
- **Robot spawn height**: `2.0m` (Safe drop to avoid interpenetration)
- **Physics Buffers**: Increased GPU rigid contact and heap capacities to handle 500+ complex hulls.

## Conclusion
While `original_production.usd` is now "solid," it is physically "bumpy" at the tile junctions. For smooth training, `no_graph_sim_final.usd` (which is manually smoothed) is superior, but the "Original" is now technically functional for 3D navigation testing. This entire sub-phase is marked **STALE** and will be revisited in Milestone 2.
