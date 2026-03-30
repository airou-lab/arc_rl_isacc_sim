# Architecture

**Analysis Date:** 2024-10-31

## Pattern Overview

**Overall:** Isaac Lab Manager-Based RL Environment

**Key Characteristics:**
- **Manager-Based Configuration**: Heavily utilizes Isaac Lab's `ManagerBasedRLEnvCfg` for defining environments, scenes, observations, actions, rewards, and terminations.
- **Physics-Driven Simulation**: Uses NVIDIA Isaac Sim (PhysX) with High Fidelity settings (200Hz - 1000Hz) for autonomous vehicle training.
- **Modularity**: Separation of robot configuration, environment setup, and RL logic (MDP).

## Layers

**Environment Layer:**
- Purpose: Defines the simulation scene, physics parameters, and environment logic.
- Location: `arcproLab/arcpro_env_cfg.py`, `arcproLab/arcpro_metric_env_cfg.py`
- Contains: `ARCProEnvCfg`, `ARCProSceneCfg`, `ObservationCfg`, `ActionCfg`, `RewardCfg`.
- Depends on: Isaac Lab API, Robot Configuration, MDP implementations.
- Used by: Training and verification scripts in `arcproLab/scripts/`.

**MDP Layer (Markov Decision Process):**
- Purpose: Implements specific RL components like rewards, observations, and termination conditions.
- Location: `arcproLab/mdp/`
- Contains: `observations.py`, `rewards.py`, `terminations.py`, `events.py`, `track_manager.py`.
- Depends on: PyTorch, Isaac Lab, NumPy.
- Used by: Environment Configuration (`arcproLab/arcpro_env_cfg.py`).

**Robot Layer:**
- Purpose: Defines the articulation, actuators, and initial state of the robot.
- Location: `arcproLab/arcpro_robot_cfg.py`
- Contains: `ArcProRobotCfg`.
- Depends on: Isaac Lab assets, USD files in `arcproLab/assets/`.
- Used by: `ARCProSceneCfg` in `arcproLab/arcpro_env_cfg.py`.

**Operational Layer (Scripts):**
- Purpose: Provides entry points for training policies and verifying performance.
- Location: `arcproLab/scripts/`, `arcproLab/verify.py`
- Contains: `train_policy.py`, `verify_policy.py`, `verify_metric.py`.
- Depends on: Environment layer, Stable Baselines 3 (for training).
- Used by: User/CLI for project execution.

## Data Flow

**RL Training Loop:**

1. **Environment Setup**: `ARCProEnvCfg` is instantiated and passed to `ManagerBasedRLEnv`.
2. **Observation Generation**: `ObservationManager` (via `mdp/observations.py`) collects data from the simulator (camera, telemetry) and `TrackManager`.
3. **Policy Inference**: Stable Baselines 3 (PPO) receives observations and predicts actions.
4. **Action Application**: `ActionManager` (via `ActionCfg`) applies steering and throttle to the robot articulation.
5. **Physics Step**: Isaac Sim steps the physics simulation.
6. **Reward Calculation**: `RewardManager` (via `mdp/rewards.py`) calculates the step reward.
7. **Termination Check**: `TerminationManager` (via `mdp/terminations.py`) checks if the episode should end.

**State Management:**
- State is primarily managed by Isaac Lab's `InteractiveScene` and `Articulation` classes, representing the physical state of the robot and environment in the USD stage.

## Key Abstractions

**TrackManager:**
- Purpose: Provides vectorized track information, waypoints, and error calculations (lateral/heading) relative to the centerline.
- Examples: `arcproLab/mdp/track_manager.py`
- Pattern: Singleton manager for waypoint-based spatial queries.

**ARCProEnvCfg:**
- Purpose: Central configuration class defining the entire RL task.
- Examples: `arcproLab/arcpro_env_cfg.py`
- Pattern: Manager-based configuration pattern.

## Entry Points

**Training Script:**
- Location: `arcproLab/scripts/train_policy.py`
- Triggers: CLI execution (`python train_policy.py`)
- Responsibilities: Initialize Isaac Sim app, wrap environment for SB3, run PPO training, save models.

**Verification Scripts:**
- Location: `arcproLab/scripts/verify_policy.py`, `arcproLab/scripts/verify_metric.py`
- Triggers: CLI execution
- Responsibilities: Load trained models, run inference in a single environment, log performance metrics.

## Error Handling

**Strategy:** Fail-fast for configuration errors; Graceful handling for simulation NaNs.

**Patterns:**
- **NaN Sanitization**: Observations are checked for NaNs and zeroed out to prevent policy crash in `arcproLab/mdp/observations.py`.
- **Config Post-init**: `__post_init__` in configuration classes ensures consistency between parameters (e.g., decimation vs render interval).

## Cross-Cutting Concerns

**Logging:** Uses TensorBoard via Stable Baselines 3 during training (`logs/ppo/`).
**Validation:** `TrackManager` validates waypoints and can resample them from USD if missing.
**Physics Settings:** High-fidelity PhysX settings (TGS solver, high iteration counts) are centralized in `ARCProEnvCfg`.

---

*Architecture analysis: 2024-10-31*
