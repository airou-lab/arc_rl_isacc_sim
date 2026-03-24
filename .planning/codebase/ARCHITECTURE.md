# Architecture

**Analysis Date:** 2024-05-14

## Pattern Overview

**Overall:** Isaac Lab Manager-Based RL Environment

**Key Characteristics:**
- **Configuration-Driven:** The entire environment (scene, observations, actions, rewards) is defined via `@configclass` objects in `arcproLab/arcpro_env_cfg.py`.
- **Modular MDP:** The Markov Decision Process (MDP) components are decoupled into specialized modules in `arcproLab/mdp/`.
- **USD-Backed Scenes:** Simulation physics and visuals are defined in NVIDIA Universal Scene Description (USD) files in `openStreetUSD/`.

## Layers

**Configuration Layer:**
- Purpose: Defines the environment structure and connects all components.
- Location: `arcproLab/arcpro_env_cfg.py`
- Contains: `ARCProSceneCfg`, `ObservationCfg`, `ActionCfg`, `RewardCfg`, `TerminationCfg`, `EventCfg`, and the main `ARCProEnvCfg`.
- Depends on: `isaaclab`, `arcpro_robot_cfg.py`, `arcproLab/mdp/*`
- Used by: Isaac Lab environment runner/trainers.

**MDP Layer:**
- Purpose: Implements the logic for reinforcement learning signals (observations, rewards, etc.).
- Location: `arcproLab/mdp/`
- Contains: Python functions for calculating telemetry, rewards, and termination conditions.
- Depends on: `isaaclab`, `torch`, `arcproLab/mdp/track_manager.py`
- Used by: `arcproLab/arcpro_env_cfg.py` via Isaac Lab managers.

**Logic Layer (Track Management):**
- Purpose: Handles track waypoints and provides vectorized spatial queries (lateral/heading errors).
- Location: `arcproLab/mdp/track_manager.py`
- Contains: `TrackManager` class and `get_track_manager` singleton factory.
- Depends on: `numpy`, `torch`, `omni.usd`, `pxr` (for sampling).
- Used by: `arcproLab/mdp/observations.py`, `arcproLab/mdp/rewards.py`, `arcproLab/generate_track.py`.

**Scene Layer:**
- Purpose: Defines the physical world and visual environment.
- Location: `openStreetUSD/`
- Contains: USD files (`arcpro_RL_open_street_sim.usd`).
- Depends on: NVIDIA Isaac Sim / Omniverse.
- Used by: `arcproLab/arcpro_env_cfg.py` (via `UsdFileCfg`).

## Data Flow

**Environment Initialization:**

1. `ARCProEnvCfg` is instantiated.
2. `Isaac Lab` loads the scene defined in `ARCProSceneCfg`, spawning the robot and the track from `openStreetUSD/arcpro_RL_open_street_sim.usd`.
3. `TrackManager` is initialized, loading waypoints from `arcproLab/mdp/track_centerline.npy`.

**Step Cycle (RL Loop):**

1. **Observations:** `get_telemetry_vector` in `arcproLab/mdp/observations.py` queries `TrackManager` for current lateral and heading errors based on robot position.
2. **Actions:** `ActionCfg` in `arcproLab/arcpro_env_cfg.py` maps policy outputs to joint positions (steering) and velocities (throttle).
3. **Rewards:** Functions in `arcproLab/mdp/rewards.py` calculate reward signals. `lateral_error_reward` pulls the error directly from the observation buffer.
4. **Terminations:** `arcproLab/mdp/terminations.py` (referenced in config) checks for exit conditions like white line contact.

**State Management:**
- Physics state is managed by NVIDIA PhysX within Isaac Sim.
- RL state (observations, rewards) is managed by `Isaac Lab` managers.
- Track metadata is managed by the `TrackManager` singleton.

## Key Abstractions

**TrackManager:**
- Purpose: Provides a bridge between the unstructured USD road meshes and the structured data needed for RL (ordered waypoints).
- Examples: `arcproLab/mdp/track_manager.py`
- Pattern: Singleton

**ManagerBasedRLEnvCfg:**
- Purpose: Isaac Lab's standard for defining RL environments as a composition of managers.
- Examples: `arcproLab/arcpro_env_cfg.py`
- Pattern: Configuration-as-Code

## Entry Points

**Environment Config:**
- Location: `arcproLab/arcpro_env_cfg.py`
- Triggers: Imported by training scripts.
- Responsibilities: Full environment specification.

**Track Generation:**
- Location: `arcproLab/generate_track.py`
- Triggers: Manual execution.
- Responsibilities: Samples road meshes from USD and persists waypoints to `arcproLab/mdp/track_centerline.npy`.

## Error Handling

**Strategy:** Fail-safe defaults for simulation stability.

**Patterns:**
- **NaN Mitigation:** MDP functions in `observations.py` and `rewards.py` explicitly check for and zero out `NaN` values to prevent training collapse.
- **Fallback Logic:** `TrackManager` includes a straight-line fallback if waypoints cannot be loaded or sampled.

---

*Architecture analysis: 2024-05-14*
