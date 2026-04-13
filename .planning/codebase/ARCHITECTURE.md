# Architecture

**Analysis Date:** 2024-04-12

## Pattern Overview

**Overall:** Modular RL Environment based on Isaac Lab's Manager-Based RL pattern, transitioned to a 1.0x true metric physics scale.

**Key Characteristics:**
- **True Physics Scaling:** All assets and physics parameters are aligned to a 1.0x metric scale (1 unit = 1 meter), ensuring realistic dynamics.
- **Road in Void:** Minimalist environment architecture where the robot exists on a track suspended in a void, facilitating clean training and strict termination logic.
- **Decoupled MDP:** Observations, rewards, and terminations are defined in modular files within `arcproLab/mdp/`, allowing for easy iteration of the RL logic.

## Layers

**Configuration Layer:**
- Purpose: Defines the environment, robot, and training parameters.
- Location: `arcproLab/arcpro_env_cfg.py`, `arcproLab/arcpro_robot_cfg.py`
- Contains: Isaac Lab config classes (`ObservationCfg`, `RewardCfg`, etc.).
- Depends on: `mdp` logic, `isaaclab` core.
- Used by: Training and verification scripts.

**Logic Layer (MDP):**
- Purpose: Implements the physics-to-RL mapping (state, reward, done).
- Location: `arcproLab/mdp/`
- Contains: Observation functions, reward terms, termination conditions, and the `TrackManager`.
- Depends on: `torch`, `isaaclab`.
- Used by: `arcproLab/arcpro_env_cfg.py`.

**Asset Layer:**
- Purpose: Provides the 3D models and environment definitions.
- Location: `openStreetUSD/`, `arcproLab/assets/`
- Contains: `.usd` and `.usda` files.
- Depends on: NVIDIA Isaac Sim / USD framework.
- Used by: `arcproLab/arcpro_env_cfg.py`.

## Data Flow

**Observation Pipeline:**

1. Raw physics data (positions, velocities, quats) is pulled from the Isaac Sim stage via `SceneEntityCfg`.
2. `mdp.observations.get_telemetry_vector` processes raw data into a 12-element telemetry vector.
3. `TrackManager` computes lateral and heading errors relative to the `track_centerline_1x.npy` waypoints.
4. Final observation vector is passed to the policy.

**State Management:**
- Environment state is managed by Isaac Lab's `ManagerBasedRLEnv`.
- Telemetry-specific state (like accumulated distance) is stored in `env.extras`.

## Key Abstractions

**TrackManager:**
- Purpose: Centralizes waypoint handling and error computation.
- Examples: `arcproLab/mdp/track_manager.py`
- Pattern: Singleton manager initialized with track-specific waypoints.

**Telemetry Protocol:**
- Purpose: Standardized 12-element vector for policy input.
- Examples: `arcproLab/mdp/observations.py`
- Pattern: Index-fixed vector (Indices 3, 4, 5, 6, 8, 9, 11).

## Entry Points

**Training Script:**
- Location: `arcproLab/scripts/train_policy.py`
- Triggers: User execution of `train.sh`.
- Responsibilities: Initializes the environment and starts the RL training loop.

**Verification Script:**
- Location: `arcproLab/scripts/verify_policy.py`
- Triggers: User execution of `verify_sim.sh`.
- Responsibilities: Loads a trained model and runs it in the simulation with telemetry visualization.

## Error Handling

**Strategy:** Fail-fast for physics anomalies, soft-reset for environment violations.

**Patterns:**
- **NaN Protection:** `get_telemetry_vector` checks for and zeros out NaNs to prevent policy explosion.
- **Strict Termination:** Resets triggered by `height_termination` (falling into void) or `white_line_contact`.

## Cross-Cutting Concerns

**Logging:** RL metrics logged to `logs/ppo/`, telemetry logged to console/UI during verification.
**Validation:** `TrackManager` validates track geometry against USD meshes during sampling.
**Scale Transition:** Centralized scaling in `arcpro_env_cfg.py` where the world is shrunk by 0.125x to match the 1.0x robot.

---

*Architecture analysis: 2024-04-12*
