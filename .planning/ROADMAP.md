# Project Roadmap: ARCPro RL Isaac Lab

## Milestone 1: Physical Fidelity (DONE)
*   **Phase 07: Modernization** [DONE]
    *   Standardize Gymnasium 1.0+.
    *   Repair USD references (Signposts/F1Tenth).
*   **Phase 08: Training Stabilization** [DONE]
    *   Fixed road detection raycasts (implemented robust direct snapping).
    *   Hardened Track USD with Triangle Meshes.
*   **Phase 09: Drive Configuration (FWD Transition)** [DONE]
    *   Modified `ActionCfg` to isolate Front-Wheel Drive.
    *   Increased effort limits to 4000.0 for 20kg mass handling.
    *   Standardized 4-joint action mapping.

## Milestone 2: Navigation & Routing (Current)
*   **Phase 10: Intersection Navigation** [ACTIVE]
    *   Restore RoadGraph logic.
    *   Implement graph-based waypoint routing.
*   **Phase 11: Multi-Agent Stability** [PLANNED]
