# Architecture

**Analysis Date:** 2024-04-23

## Pattern Overview

**Overall:** Manager-Based Reinforcement Learning Environment (Isaac Lab) with a Hierarchical Path-Planning Policy (HPPP).

**Key Characteristics:**
- **Modular MDP:** Markov Decision Process logic is decoupled into managers (Observations, Rewards, Terminations, Events).
- **Metric Scale (1.0x):** All physics and geometry are calibrated to a 1.0x metric scale (1 unit = 1 meter).
- **Hierarchical Control:** High-level navigation "Intent" is separated from low-level lane-following "Execution".

## Layers

**Simulation Layer:**
- Purpose: Handles physics (PhysX), rendering (Omniverse RTX), and USD scene management.
- Location: `openStreetUSD/`, `arcproLab/assets/`
- Contains: `no_graph_sim_clean_1x.usda`, `F1Tenth_Metric.usd`
- Depends on: NVIDIA Isaac Sim

**Environment Layer:**
- Purpose: Bridges simulation to RL algorithms using the Isaac Lab framework.
- Location: `arcproLab/arcpro_env_cfg.py`
- Contains: `ARCProEnvCfg`, `ARCProSceneCfg`
- Depends on: Simulation Layer, MDP Layer

**MDP (Manager) Layer:**
- Purpose: Defines the logic for observations, rewards, and terminations.
- Location: `arcproLab/mdp/`
- Contains: `observations.py`, `rewards.py`, `terminations.py`, `track_manager.py`, `road_graph.py`
- Depends on: Simulation Layer (for state extraction)

**Policy Layer:**
- Purpose: Neural network that maps observations to actions.
- Location: `arcproLab/policy_stack/`
- Contains: `hierarchical_policy.py`, `fusion_policy.py`
- Depends on: MDP Layer (via telemetry vector and images)

## Data Flow

**Inference Loop:**

1. **Extraction:** `TrackManager` and `RoadGraph` extract geometric/navigation state from USD stage.
2. **Observation:** `observations.py` constructs a 12-element telemetry vector + RGB image.
3. **Brain:** `HierarchicalPathPlanningPolicy` processes telemetry and image to produce steering/throttle.
4. **Action:** `arcproLab/arcpro_env_cfg.py` applies actions to robot actuators.
5. **Update:** Simulation steps; `rewards.py` and `terminations.py` evaluate the new state.

**State Management:**
- **Simulation State:** Managed by Isaac Sim/PhysX.
- **Track State:** Managed by `TrackManager` (caches boundary points and waypoints).
- **Navigation State:** Managed by `RoadGraph` (Turn Tokens).

## Key Abstractions

**TrackManager:**
- Purpose: Provides robust boundary detection and error calculation.
- Examples: `arcproLab/mdp/track_manager.py`
- Pattern: Singleton/Manager (via `get_track_manager`).

**RoadGraph:**
- Purpose: Manages high-level navigation decisions (Intent).
- Examples: `arcproLab/mdp/road_graph.py`
- Pattern: Intent-based control.

**Telemetry Vector:**
- Purpose: Standardized 12-element interface between environment and policy.
- Examples: Defined in `arcproLab/mdp/observations.py`.

## Entry Points

**Training Script:**
- Location: `arcproLab/scripts/train_policy.py`
- Triggers: User execution via `python train_policy.py`.
- Responsibilities: Configures environment, initializes SB3 agent, runs training loop.

**Verification Script:**
- Location: `arcproLab/scripts/verify_policy.py`
- Triggers: User execution.
- Responsibilities: Loads a trained model and runs it in the environment for visual/metric audit.

## Error Handling

**Strategy:** Fail-fast for configuration; Robust defaults for simulation (NaN handling).

**Patterns:**
- **NaN Masking:** Found in `observations.py` and `rewards.py` to prevent policy collapse.
- **Graceful Sync:** `TrackManager.ensure_synced` handles delayed USD stage loading.

## Cross-Cutting Concerns

**Logging:** Tensorboard via SB3 callbacks.
**Validation:** `arcproLab/scripts/` contains multiple `verify_*.py` scripts for unit-testing different system components (spawn, markers, physics).
**Metric Scaling:** Consistent 1.0x scale enforced via `ARCProSceneCfg` and asset scaling.

---

*Architecture analysis: 2024-04-23*
