# Architecture

**Analysis Date:** 2024-10-24

## Pattern Overview

**Overall:** Manager-Based Reinforcement Learning (Isaac Lab)

**Key Characteristics:**
- **Configuration-Driven:** Environment and robot properties are defined using structured configuration classes (`ARCProEnvCfg`, `ArcProRobotCfg`).
- **MDP-Centric:** Core RL logic (observations, rewards, etc.) is decoupled from the simulation engine through manager-based abstractions.
- **Metric Scaling:** The environment is standardized to 1:1 metric scale (meters) for all physical calculations and robot dimensions.
- **Grounded Physics:** Uses a "drop-in" approach where the robot is spawned above the track with gravity enabled (`fix_root_link=False`), allowing for natural physics interactions.

## Layers

**Configuration Layer:**
- Purpose: Defines the simulation scene, assets, and MDP structure.
- Location: `arcproLab/arcpro_env_cfg.py`, `arcproLab/arcpro_robot_cfg.py`
- Contains: `ARCProEnvCfg`, `ArcProRobotCfg`, `ARCProSceneCfg`.
- Depends on: `isaaclab.utils.configclass`, `isaaclab.envs`, `isaaclab.assets`.
- Used by: `ManagerBasedRLEnv`, training and verification scripts.

**MDP Layer (Markov Decision Process):**
- Purpose: Implements the RL interface: how the agent sees, acts, and is rewarded.
- Location: `arcproLab/mdp/`
- Contains: `observations.py`, `rewards.py`, `terminations.py`, `events.py`.
- Depends on: `torch`, `isaaclab.managers`.
- Used by: `ARCProEnvCfg` (to map functions to managers).

**Service Layer (Track Management):**
- Purpose: Provides geometric and topological information about the racing track.
- Location: `arcproLab/mdp/track_manager.py`
- Contains: `TrackManager` class.
- Depends on: `torch`, `numpy`, `omni.usd` (for sampling).
- Used by: `mdp/observations.py` to calculate lateral and heading errors.

**Asset Layer:**
- Purpose: Provides 3D models and physics properties for the simulation.
- Location: `arcproLab/assets/robot/`, `openStreetUSD/`
- Contains: `F1Tenth_Metric.usd`, `no_graph_sim_cleaned.usd`.
- Depends on: NVIDIA Omniverse USD format.
- Used by: Configuration Layer (`ARCProSceneCfg`).

## Data Flow

**Inference Loop:**

1. **Sim Update:** Isaac Sim advances physics and sensors.
2. **Observations:** `ObservationManager` calls `get_telemetry_vector` (in `mdp/observations.py`), which queries `TrackManager` for track errors.
3. **Policy:** Policy (SB3 or manual) receives the telemetry vector and outputs actions.
4. **Actions:** `ActionManager` applies steering and throttle values to the robot's joints (`Joint_Steer_*`, `Joint_Drive_*`).
5. **Physics:** `PhysX` engine calculates new robot pose based on joint forces and gravity.

**State Management:**
- **Global State:** Managed by `ManagerBasedRLEnv`.
- **Track State:** Managed by `TrackManager` (singleton), which loads/samples track waypoints once.

## Key Abstractions

**TrackManager:**
- Purpose: Centralizes track coordinate logic and error calculation.
- Examples: `arcproLab/mdp/track_manager.py`
- Pattern: Singleton/Service.

**ArticulationCfg:**
- Purpose: Defines the robot's physical structure, joints, and actuators.
- Examples: `arcproLab/arcpro_robot_cfg.py`
- Pattern: Isaac Lab Asset Configuration.

## Entry Points

**Training Script:**
- Location: `arcproLab/scripts/train_policy.py`
- Triggers: CLI execution via `python`.
- Responsibilities: Initializes environment, wraps for SB3, runs the PPO training loop.

**Verification Script:**
- Location: `arcproLab/verify.py`
- Triggers: CLI execution or `run_gui_verify.sh`.
- Responsibilities: Visual verification of robot spawning and basic movement in the environment.

## Error Handling

**Strategy:** Fail-fast on configuration errors, robust recovery for simulation NaNs.

**Patterns:**
- **NaN Sanitization:** Observations are checked for NaNs in `mdp/observations.py` and zeroed out to prevent policy collapse.
- **Fallback Logic:** `TrackManager` uses a fallback straight-line track if waypoint files or USD sampling fails.

## Cross-Cutting Concerns

**Logging:** Uses `TensorBoard` via SB3 during training.
**Validation:** Robot spawning and physics are validated in `verify_spawn.py` and `verify.py`.
**Scaling:** All coordinates are in meters (Metric Transition Phase).

---

*Architecture analysis: 2024-10-24*
