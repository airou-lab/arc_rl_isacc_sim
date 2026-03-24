# Verification History

## Phase 7: Surgical Calibration (March 2026)
- **Status:** STABILIZED
- **Baseline Asset:** `openStreetUSD/no_graph_sim_cleaned.usd`
- **Scale:** 1.0x (Global)
- **Proportions:** RC Car sized correctly relative to lane width.
- **Physics Foundation:** Ground Plane active; 3D Mesh Clipping identified as subsequent blocker.
- **Visuals:** Full color and markings restored from `no_graph_sim`.

---

## Legacy Verification History

> **STATUS:** Historical verification records. Current verification is moved to Isaac Lab Phase 3.

## Key Milestones (Legacy)
1.  **ROS2 Bridge (Verified):** Confirmed `AckermannDriveStamped` connectivity between Isaac Sim and external ROS2 nodes.
2.  **Physics Stability (Verified):** Confirmed the 34-joint robot remains stable with 50 damping / 1000 stiffness overrides.
3.  **Metric Scale (REPAIRED):** Verified chassis mesh is 24cm x 14cm. Fixed joint local positions to match metric 0.25m wheelbase.
4.  **Vision (Verified):** Confirmed 160x90 frame capture via Replicator Annotators.
