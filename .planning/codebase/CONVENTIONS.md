# Coding Conventions

**Analysis Date:** 2024-10-24

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
- UPPER_CASE: `ARCPRO_ROBOT_CFG`, `WAYPOINT_NORM_SCALE`.

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

**Robot Scale:**
- Use **0.5x scale** for all F1Tenth robot assets to match the metric environment.

**Drive Configuration:**
- **Front-Wheel Drive (FWD)**: Only `Joint_Drive_FL` and `Joint_Drive_FR` should be used for acceleration.

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
- Use of `warnings.warn` for sim-related warnings that shouldn't crash training.
- `try-except` blocks around complex SB3-contrib logic (e.g., RNN state handling).

## Logging

**Framework:** TensorBoard (integrated with SB3) and standard Python `logging` / `print`.

## Module Design

**Exports:** Explicitly defining `ARCPRO_ROBOT_CFG` in config files.

---

*Convention analysis: 2024-10-24*
