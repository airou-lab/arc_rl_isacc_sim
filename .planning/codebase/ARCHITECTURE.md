# Architecture

**Analysis Date:** 2024-11-20

## Pattern Overview

**Overall:** Manager-Based Reinforcement Learning Environment (Isaac Lab pattern)

**Key Characteristics:**
- Configuration-driven environment and robot definitions.
- Decoupled MDP terms (observations, rewards, actions, terminations).
- Hierarchical policy structure (referenced in `train_policy.py`).

## Layers

**Simulation Layer:**
- Purpose: Handles physics, rendering, and USD stage management.
- Location: `openStreetUSD/` (assets), `arcproLab/arcpro_env_cfg.py` (scene config).
- Contains: USD files, light configs, ground planes.
- Depends on: NVIDIA Isaac Sim.
- Used by: MDP Layer.

**MDP Layer (Markov Decision Process):**
- Purpose: Defines how the agent interacts with the simulation.
- Location: `arcproLab/mdp/`
- Contains: `actions.py`, `observations.py`, `rewards.py`, `terminations.py`, `events.py`.
- Depends on: Simulation Layer, `TrackManager`.
- Used by: Policy Layer.

**Policy Layer:**
- Purpose: Implements the neural networks and RL algorithms.
- Location: `arcproLab/policy_stack/`
- Contains: `policies/`, `agent/`, `baselines/`.
- Depends on: MDP Layer.
- Used by: Training and Verification scripts.

**Infrastructure Layer:**
- Purpose: Utilities for track management, road graphs, and analytics.
- Location: `arcproLab/mdp/track_manager.py`, `arcproLab/mdp/road_graph.py`, `arcproLab/mdp/visual_analytics.py`.
- Contains: Pathfinding, boundary detection, visualization markers.

## Data Flow

**Standard RL Loop:**

1. **Step:** `isaaclab` triggers a simulation step.
2. **Observe:** `observations.py` extracts state (e.g., relative waypoint positions, velocities).
3. **Act:** Policy computes actions (e.g., steering, throttle) based on observations.
4. **Apply:** `actions.py` applies torques/velocities to the robot joints.
5. **Reward/Terminate:** `rewards.py` and `terminations.py` calculate feedback and reset conditions.

**Track Initialization Flow:**

1. `TrackManager` loads centerline waypoints from `track_centerline_1x.npy`.
2. `TrackManager` checks for cached boundaries in `track_boundaries_1x.npz`.
3. If cache miss, `TrackManager` scans USD stage for markers (slow).
4. `RoadGraph` builds a connectivity map for intersection navigation.

## Key Abstractions

**TrackManager:**
- Purpose: Centralized access to track geometry and boundaries.
- Examples: `arcproLab/mdp/track_manager.py`
- Pattern: Singleton-like utility injected into MDP terms.

**ManagerBasedRLEnvCfg:**
- Purpose: Declarative configuration of the entire RL environment.
- Examples: `arcproLab/arcpro_env_cfg.py`
- Pattern: Configuration-as-code.

## Entry Points

**Training Script:**
- Location: `arcproLab/scripts/train_policy.py`
- Triggers: User execution via CLI.
- Responsibilities: Launches Isaac Sim, initializes env, starts SB3 training loop.

**Verification Script:**
- Location: `arcproLab/scripts/verify_policy.py`
- Triggers: User execution via CLI.
- Responsibilities: Loads a trained model and runs it in the simulation for evaluation.

## Error Handling

**Strategy:** Defensive checks and caching.

**Patterns:**
- Cache loading with fallback: `TrackManager.load_cache()`.
- Wait-for-sync loops for USD assets.

## Cross-Cutting Concerns

**Logging:** Tensorboard for metrics, prefixed stdout for component logs.
**Validation:** `arcproLab/scripts/verify_*.py` for functional validation.
**Authentication:** N/A (Local simulation).

---

*Architecture analysis: 2024-11-20*
