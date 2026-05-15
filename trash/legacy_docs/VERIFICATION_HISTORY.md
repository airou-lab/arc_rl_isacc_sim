# Verification History

## Phase 8: v1.0 Release (March 25, 2026)
- **Status:** VERIFIED & RELEASED
- **Baseline Asset:** `openStreetUSD/no_graph_sim_cleaned.usd`
- **Scale:** 20.0x (Internal Articulation) / 1.0x (Global Reference)
- **Stability:** Solid performance at 1000Hz (dt=0.001) in Isaac Lab.
- **Autonomous Driving:** Verified SB3 policy inference. Robot successfully completes laps in the Isaac Lab environment with RGB camera and telemetry feedback.
- **Metric Accuracy:** Confirmed 403mm length and 25cm wheelbase mapping.

---

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
