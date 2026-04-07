# Coding Conventions

**Analysis Date:** 2025-04-18

## Naming Patterns

**Files:**
- snake_case: `arcpro_env_cfg.py`, `track_manager.py`.

**Functions:**
- snake_case: `compute_reward()`, `get_telemetry_vector()`.

**Variables:**
- snake_case: `joint_pos`, `throttle_limit`.

**Types / Classes:**
- PascalCase: `ArcProRobotCfg`, `HierarchicalPathPlanningPolicy`, `TrackManager`.

**Constants:**
- UPPER_CASE: `ARCPRO_LAB_DIR`, `USD_DIR`.

## Telemetry Protocol (12-Float)

All observation vectors returned by `get_telemetry_vector()` in `arcproLab/mdp/observations.py` must follow this mapping:

| Index | Name | Units | Description |
|-------|------|-------|-------------|
| 0 | TURN_TOKEN | {-1, 0, 1} | Navigation intent from high-level worker |
| 1 | GO_SIGNAL | {0.0, 1.0} | Scheduler-controlled stop/go |
| 2 | GOAL_DIST | meters | Distance to target (often masked) |
| 3 | SPEED | m/s | Forward vehicle speed |
| 4 | YAW_RATE | rad/s | Angular velocity around Z-axis |
| 5 | LAST_STEER | [-1, 1] | Previous steering action |
| 6 | LAST_THR | [-1, 1] | Previous throttle action |
| 7 | LAST_BRK | [0, 1] | Previous brake action |
| 8 | LAT_ERR | meters | Lateral deviation from absolute waypoint |
| 9 | HDG_ERR | radians | Heading deviation from absolute waypoint |
| 10 | KAPPA | 1/m | Local path curvature |
| 11 | DS | meters | Cumulative odometry |

## Physical Conventions

**Baseline Robot Scale (v1.2-dev):**
- **8.0x metric scale override** in `arcpro_env_cfg.py`. This is the current stable baseline for simulation stability and visual alignment.
- **0.125 Normalization**: All telemetry inputs (Speed, Lateral Error) are scaled by `0.125` in `observations.py` to keep the policy metric-aligned.

**Drive Configuration:**
- **4WD (Four-Wheel Drive)**: All wheels (`Joint_Drive_.*`) are used for acceleration to maintain traction for the 20kg chassis.

**Mass:**
- Standardized at **20.0kg** for the F1Tenth_Metric robot to ensure realistic inertia and suspension behavior.

**Waypoint Alignment:**
- Use **Absolute Waypoint Alignment**. Do not shift waypoints to origin; track them relative to world coordinates for multi-agent and large-map compatibility.

## Code Style

**Formatting:**
- flake8 (implied by `requirements.txt`).
- Standard Python (PEP 8) with emphasis on `@configclass` usage for Isaac Sim.

**Linting:**
- flake8 for syntax and style checking.

## Import Organization

**Order:**
1. Standard library (`os`, `sys`, `math`).
2. Major third-party libraries (`torch`, `numpy`).
3. Isaac Sim / IsaacLab imports (`isaaclab.sim`, `isaaclab.envs`).
4. Local project imports (`arcpro_robot_cfg`, `mdp.observations`).

## Error Handling

**Patterns:**
- NaN handling in observation tensors to prevent model collapse.
- Default fallback waypoints in `TrackManager` if USD sampling fails.

## Logging

**Framework:** TensorBoard (integrated with SB3) and standard Python `logging` / `print`.

## Module Design

**Exports:** Explicitly defining `ARCPRO_ROBOT_CFG` in config files for external consumption.

---

*Convention analysis: 2025-04-18*
