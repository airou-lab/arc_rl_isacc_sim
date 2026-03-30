# Phase 07 Context: Revert to True Physics Mode

## Goal
The goal of this phase is to revert the simulation environment to use a true-to-scale (1.0x metric) F1Tenth robot and a proportionally scaled OpenStreetMap track. This will remove previous workarounds (Giant Scale, high physics solver iterations) for a more realistic and transferable simulation.

## Implementation Plan
The execution steps for this phase are detailed in the [07-01-PLAN.md](./07-01-PLAN.md) file.

## Background
Previous phases introduced "Giant Scale" (20x robot, 1x track) and highly tuned physics solver settings (e.g., high iteration counts, damping, stiffness overrides) as workarounds for stability issues like the "Collision Sandwich" and a 100x joint offset unit mismatch in the original USD. Our recent investigation has confirmed that with proper scaling of both robot and track, and a corrected robot USD, these workarounds are no longer necessary. A 1.0x metric simulation has been successfully tested and proved stable with standard PhysX settings.

## Objectives
1.  **Switch to Metric Robot Asset:** Configure `arcpro_robot_cfg.py` to use the newly generated `F1Tenth_Metric.usd` (1.0x scale).
2.  **Scale Down Track Asset:** Modify `arcpro_env_cfg.py` to apply a scale factor of `0.0825` to the `no_graph_sim_cleaned.usd` track, bringing its lanes to realistic metric dimensions (~3.5m).
3.  **Adjust Initial Robot Spawn:** Update the robot's initial position (`init_state.pos`) in `arcpro_env_cfg.py` to reflect the metric scale (e.g., spawn it closer to the ground, relative to the track's new height).
4.  **Remove Physics Solver Overrides:** In `arcpro_robot_cfg.py` and `arcpro_env_cfg.py`, set physics solver iteration counts and any other custom PhysX parameters back to standard/default values.
5.  **Remove Robot Stability Event:** Remove or disable the `setup_robot_stability` event in `mdp/events.py` as it will no longer be necessary.
6.  **Adjust Camera Offset:** Update the `tiled_camera` offset in `arcpro_env_cfg.py` to match the 1.0x metric robot scale.
7.  **Verify Stability and Performance:** Conduct thorough testing to ensure the metric simulation is stable, performs well, and that the robot can still navigate the track correctly.
8.  **Update Documentation:** Reflect the new "True Physics" mode and removed workarounds in relevant documentation files (e.g., `docs/RESEARCH.md`, `docs/STATE.md`, `docs/RL_ISAAC_SIM.md`).

## Decisions

### 1. Track Scaling Factor
*   **Decision:** The track asset (`openStreetUSD/no_graph_sim_cleaned.usd`) will be scaled down by a factor of **0.0825** (e.g., in `arcproLab/arcpro_env_cfg.py`). This factor is considered precise enough for now to bring the lane widths to a realistic metric scale (~3.5m).

### 2. Robot Initial Spawn Height
*   **Decision:** The robot's initial spawn height will be set to **0.5m** above the track. This is deemed optimal to prevent initial collisions or clipping while minimizing excessive bouncing.

### 3. Physics Solver Settings
*   **Decision:** Standard PhysX solver settings will be used. This means `solver_position_iteration_count=4` and `solver_velocity_iteration_count=1`. The existing `TGS` solver type will be maintained.

### 4. Configuration File Modification Strategy
*   **Decision:** The existing configuration files, `arcproLab/arcpro_env_cfg.py` and `arcproLab/arcpro_robot_cfg.py`, will be directly modified. This is acceptable as the changes are being made on a dedicated git branch (`feature/true-physics-revert`).

### 5. Robot Asset
*   **Decision:** The simulation will use the `F1Tenth_Metric.usd` asset (1.0x metric scale), which has already been generated. This will be specified in `arcproLab/arcpro_robot_cfg.py`.

### 6. Robot Stability Event
*   **Decision:** The `setup_robot_stability` event in `arcproLab/mdp/events.py` will be removed or disabled, as the high damping and stiffness overrides are no longer required for stability in the metric environment.

### 7. Camera Offset
*   **Decision:** The `tiled_camera` offset in `arcproLab/arcpro_env_cfg.py` will be adjusted to `(0.28, 0.0, 0.16)` to correctly position the camera relative to the 1.0x metric robot.

## Code Context

*   **Files to Modify:**
    *   `arcproLab/arcpro_env_cfg.py`
    *   `arcproLab/arcpro_robot_cfg.py`
    *   `arcproLab/mdp/events.py`
*   **Relevant Assets:**
    *   `arcproLab/assets/robot/F1Tenth_Metric.usd` (newly generated)
    *   `openStreetUSD/no_graph_sim_cleaned.usd` (track)

## Dependencies
*   Completion of the `F1Tenth_Metric.usd` generation. (Already done)

## Success Criteria
*   The simulation successfully launches without errors using `arcpro_env_cfg.py`.
*   The robot appears at a realistic 1.0x scale relative to the track.
*   The robot remains stable during a drop test and while driving.
*   Physics performance (FPS) is maintained or improved compared to the "Giant Scale" setup.
*   All previous workarounds for scaling and physics are removed from the configuration.

## Discussion Points
*   Are there any unforeseen side effects of drastically scaling down the track?
*   Should we introduce a new environment configuration (`arcpro_metric_env_cfg.py`) instead of modifying the existing `arcpro_env_cfg.py` for easier comparison/fallback? (Current plan is to modify existing, but this can be discussed).
*   Any other changes needed in `mdp/events.py` or other files due to the scale change?
