# Architecture

**Analysis Date:** 2025-05-15

## Pattern Overview

**Overall:** Modular RL Environment based on Isaac Lab's Manager-Based RL pattern, fully transitioned to a 1.0x true metric physics scale.

**Key Characteristics:**
- **True Metric Scaling:** The robot is defined at 1.0x scale (1 unit = 1 meter). The environment USD is scaled to 0.125x to achieve metric parity with the robot, ensuring realistic mass and inertia dynamics.
- **Road in Void:** Minimalist environment architecture (`no_graph_sim_clean_1x.usda`) where the robot exists on a track suspended in a void, facilitating clean training and strict termination logic.
- **Vision-Centric Navigation:** Navigation is driven by a tiled camera sensor with integrated FOV-based termination to ensure the agent remains within visual operational limits.

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
3. `TrackManager` computes lateral and heading errors relative to the `track_centerline_1x.npy` waypoints using environment-relative positions (`root_pos_w - env_origins`).
4. Final observation vector is passed to the policy, with target lateral error shifted by 0.5625m to center the robot in the right lane.

**State Management:**
- Environment state is managed by Isaac Lab's `ManagerBasedRLEnv`.
- Telemetry-specific state (like accumulated distance) is stored in `env.extras`.

## Key Abstractions

**TrackManager:**
- Purpose: Centralizes waypoint handling and error computation in metric units.
- Examples: `arcproLab/mdp/track_manager.py`
- Pattern: Singleton manager initialized with 1x-scaled waypoints (`track_centerline_1x.npy`).

**Telemetry Protocol:**
- Purpose: Standardized 12-element vector for policy input.
- Examples: `arcproLab/mdp/observations.py`
- Pattern: Index-fixed vector (Index 3: Speed, 4: Yaw Rate, 5-6: Actions, 8: Lat Err, 9: Head Err, 11: Distance).

## Entry Points

**Training Script:**
- Location: `arcproLab/scripts/train_policy.py`
- Triggers: User execution of `train.sh`.
- Responsibilities: Initializes the environment and starts the RL training loop using SB3 PPO.

**Verification Script:**
- Location: `arcproLab/scripts/verify_policy.py` and `arcproLab/scripts/verify_live.py`.
- Triggers: User execution of `verify_sim.sh` or `run_gui_verify.sh`.
- Responsibilities: Loads a trained model and runs it in the simulation with telemetry visualization and FOV monitoring.

## Error Handling

**Strategy:** Fail-fast for physics anomalies, strict termination for environment violations.

**Patterns:**
- **NaN Protection:** `get_telemetry_vector` checks for and zeros out NaNs to prevent policy explosion.
- **FOV Termination:** `fov_visibility_termination` resets the environment if the robot's velocity vector points outside the camera's horizontal FOV.
- **Strict Lane Termination:** `white_line_contact` triggers reset if lateral error exceeds 2.7m or is less than 0.225m (right lane boundaries).

## Cross-Cutting Concerns

**Logging:** RL metrics logged to `logs/ppo/`, telemetry logged to console/UI during verification.
**Validation:** `TrackManager` validates track geometry against 1x waypoints.
**Scale Parity:** Centralized scaling in `arcpro_env_cfg.py` where the world is scaled to 0.125x to match the 1.0x robot's metric properties (20kg mass, metric dimensions).

---

*Architecture analysis: 2025-05-15*
