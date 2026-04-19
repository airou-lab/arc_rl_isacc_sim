# Phase 11 Discussion Records

## 1. Protocol v2 & Retraining
- **Decision:** Shift from Coordinate-based (X, Y) to Navigation-based (Token, Signal) telemetry.
- **Rationale:** The robot is being retrained for 5M steps; absolute coordinates are no longer needed as the HPPP relies on vision and relative lateral error. Retraining eliminates any "alignment" risk.

## 2. Policy Architecture Preservation
- **Decision:** The `HierarchicalPathPlanningPolicy` and its sub-modules (FusionExtractor, LSTM) will be integrated with **zero logic changes**.
- **Rationale:** Prioritize research consistency and architecture integrity. Higher training time (LSTM overhead) is an accepted trade-off.

## 3. Submodule Integration
- **Decision:** Use **Git Submodules** at `arcproLab/policy_stack` to link the reference repo.
- **Rationale:** Provides version locking and cross-system portability (via `git clone --recursive`). Fits into the existing IDE-based workflow.

## 4. Control Logic & "Uplifting"
- **Decision:** Implement a `CombinedDriveAction` in the MDP layer: `speed = throttle * (1.0 - brake)`.
- **Rationale:** Handles the HPPP's 3-dim action output while keeping the physics bridge simple. This creates a "translation layer" that makes it easy to refactor the robot to a high-fidelity model later without changing the brain.

## 5. Waypoint Supervision (Auxiliary Loss)
- **Decision:** Enable the `WaypointTrackingWrapper` for supervised imitation loss.
- **Rationale:** Teaches the Planning Head "how to think" geometrically. This serves as a direct architectural bridge to future UFLD lane-detection integration.
