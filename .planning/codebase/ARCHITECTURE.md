# Architecture

**Analysis Date:** 2024-10-24

## Pattern Overview

**Overall:** Manager-Based Reinforcement Learning Environment

The ARCPro system is built on top of the NVIDIA Isaac Lab framework, following a manager-based design pattern. It decouples the simulation scene, the robot configuration, and the Markov Decision Process (MDP) logic (observations, rewards, terminations, events) into distinct configuration and logic modules.

**Key Characteristics:**
- **Configuration-Driven**: Environments and robots are defined using `@configclass` objects, allowing for modular and swappable simulation parameters.
- **MDP Separation**: RL logic (observations, rewards, etc.) is isolated from the simulation setup, typically living in the `mdp` subpackage.
- **USD-Coupled Geometry**: Track-following logic is dynamically coupled to USD stage geometry through runtime sampling of road meshes.

## Layers

**Environment Configuration Layer:**
- Purpose: Defines the simulation scene, robot assets, sensors, and RL managers.
- Location: `arcproLab/`
- Contains: `arcpro_env_cfg.py`, `arcpro_robot_cfg.py`
- Depends on: `isaaclab`, `arcproLab/mdp/`, `openStreetUSD/`
- Used by: RL training and verification scripts in `arcproLab/scripts/`

**MDP Logic Layer:**
- Purpose: Implements the functions for observations, rewards, terminations, and events.
- Location: `arcproLab/mdp/`
- Contains: `observations.py`, `rewards.py`, `terminations.py`, `events.py`
- Depends on: `isaaclab`, `arcproLab/mdp/track_manager.py`
- Used by: `arcproLab/arcpro_env_cfg.py`

**Track Management Layer:**
- Purpose: Provides a bridge between static USD geometry and dynamic RL observations.
- Location: `arcproLab/mdp/track_manager.py`
- Contains: `TrackManager` class for waypoint sampling and error calculation.
- Depends on: `omni.usd`, `pxr.Usd`, `torch`
- Used by: `arcproLab/mdp/observations.py`

**USD Asset Layer:**
- Purpose: Stores the 3D models and physics properties for the environment and robot.
- Location: `openStreetUSD/`, `arcproLab/assets/robot/`
- Contains: `no_graph_sim_final.usd`, `F1Tenth_Metric.usd`
- Depends on: Nothing (external assets)
- Used by: `arcproLab/arcpro_env_cfg.py`, `arcproLab/arcpro_robot_cfg.py`

## Data Flow

**Observation Flow:**

1. **Simulation Step**: PhysX updates the robot state in the USD stage.
2. **State Extraction**: Isaac Lab extracts root positions and velocities.
3. **Track Processing**: `TrackManager` (in `mdp/track_manager.py`) finds the closest waypoint to the robot's world position.
4. **Error Computation**: Lateral and heading errors are calculated relative to the sampled track centerline.
5. **Observation Vector**: `mdp/observations.py` assembles the telemetry vector (speed, yaw rate, errors, etc.) and returns it to the RL algorithm.

**Action Flow:**

1. **Policy Inference**: The RL policy generates steering and throttle commands.
2. **Action Mapping**: `ActionCfg` (in `arcpro_env_cfg.py`) maps these to joint position (steering) and joint velocity (throttle) targets.
3. **Actuator Update**: `ImplicitActuatorCfg` (in `arcpro_robot_cfg.py`) applies forces to the USD articulation joints.

**State Management:**
- Simulation state is managed by `isaaclab.envs.ManagerBasedRLEnv`.
- Waypoint state is managed by the `_TRACK_MANAGER` singleton in `mdp/track_manager.py`.

## Key Abstractions

**TrackManager:**
- Purpose: Generates a mathematical representation of the track from arbitrary USD mesh geometry.
- Examples: `arcproLab/mdp/track_manager.py`
- Pattern: Singleton/Utility Manager

**ARCProEnvCfg:**
- Purpose: Top-level configuration that aggregates all managers and scene entities.
- Examples: `arcproLab/arcpro_env_cfg.py`
- Pattern: Configuration Object

## Entry Points

**Training Script:**
- Location: `arcproLab/scripts/train_policy.py`
- Triggers: CLI execution (`python train_policy.py`)
- Responsibilities: Initializes the `ARCProEnvCfg`, starts the training loop (likely via `skrl` or `stable-baselines3`).

**Verification Script:**
- Location: `arcproLab/scripts/verify_policy.py`
- Triggers: CLI execution (`python verify_policy.py`)
- Responsibilities: Loads a trained model and runs it in the environment for evaluation.

## Error Handling

**Strategy:** Fail-soft with fallbacks for simulation anomalies.

**Patterns:**
- **NaN Masking**: Observations and rewards explicitly mask NaNs (in `mdp/observations.py` and `mdp/rewards.py`) to prevent gradient explosion.
- **Waypoint Fallback**: `TrackManager` generates a default straight-line track if USD sampling fails.

## Cross-Cutting Concerns

**Logging:** Handled by Isaac Lab and the RL framework (Tensorboard/WandB).
**Validation:** Assets are audited via scripts in `arcproLab/scripts/audit_assets.py`.
**Scaling:** The system supports multi-environment parallelization (`num_envs` in `ARCProSceneCfg`).

---

*Architecture analysis: 2024-10-24*
