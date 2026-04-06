# Project Roadmap: ARCPro RL Isaac Lab

## Milestone 1: Physical Fidelity (Current)
*   **Phase 07: Modernization** [DONE]
    *   Standardize Gymnasium 1.0+.
    *   Repair USD references (Signposts/F1Tenth).
*   **Phase 08: Training Stabilization** [ACTIVE]
    *   Fix road detection raycasts (resolve Z=10.0 fallback).
    *   Verify torque vs. 20kg mass.
*   **Phase 09: Drive Configuration (FWD Transition)** [NEW]
    *   Modify `ActionCfg` to isolate Front-Wheel Drive.
    *   Verify 20kg mass handling under FWD acceleration.
    *   Re-tune reward signals for FWD dynamics.

## Milestone 2: Navigation & Routing
*   **Phase 10: Intersection Navigation** [PLANNED]
    *   Restore RoadGraph logic.
    *   Implement graph-based waypoint routing.
*   **Phase 11: Multi-Agent Stability** [PLANNED]
