# Architecture

**Analysis Date:** 2024-10-24

## Pattern Overview

**Overall:** Manager-Based Reinforcement Learning Environment (IsaacLab pattern).

**Key Characteristics:**
- **Modular MDP Components**: Rewards, observations, terminations, and events are decoupled into separate modules in `arcproLab/mdp/`.
- **Configuration-Driven Simulation**: All environment and robot parameters are defined via `@configclass` structures in `arcpro_env_cfg.py` and `arcpro_robot_cfg.py`.
- **Hierarchical Policy Architecture**: A two-stage policy that separates path planning (waypoint prediction) from low-level Ackermann control.

## Layers

**Configuration Layer:**
- Purpose: Defines the scene, robot properties, and training hyperparameters.
- Location: `arcproLab/arcpro_env_cfg.py`, `arcproLab/arcpro_robot_cfg.py`.
- Contains: IsaacLab configuration classes.
- Depends on: `isaaclab.utils.configclass`.

**MDP Layer (Markov Decision Process):**
- Purpose: Implements the RL interface (Observations, Rewards, etc.).
- Location: `arcproLab/mdp/`.
- Contains: `observations.py`, `rewards.py`, `terminations.py`, `events.py`, `track_manager.py`.
- Used by: `arcpro_env_cfg.py`.

**Policy Layer:**
- Purpose: Implements neural network architectures and RL algorithms.
- Location: `arcproLab/policy_stack/`.
- Contains: `policies/hierarchical_policy.py`, `agent/agent_node.py`.
- Depends on: `stable_baselines3`, `sb3_contrib`, `torch`.

**Asset Layer:**
- Purpose: Stores USD assets and pre-computed track data.
- Location: `arcproLab/assets/`, `openStreetUSD/`.
- Contains: `.usd` files for robot and track, `.npy` for waypoints.

## Data Flow

**Training Loop:**

1. **Environment Initialization**: `arcpro_env_cfg.py` loads the track (`no_graph_sim_final.usd`) and robot (`F1Tenth_Metric.usd`).
2. **Observation Gathering**: `mdp/observations.py` computes telemetry (12-float protocol) and vision input.
3. **Policy Inference**: `policy_stack/policies/hierarchical_policy.py` predicts waypoint deviations and converts them to Ackermann commands.
4. **Action Application**: Commands (steer, throttle) are sent back to the Isaac Sim actuators.
5. **Reward & Termination**: `mdp/rewards.py` and `mdp/terminations.py` evaluate the state for training signal.

**State Management:**
- Handled by Isaac Sim's physics state and managed via IsaacLab's `ManagerBasedRLEnv`.

## Key Abstractions

**TrackManager:**
- Purpose: Handles waypoint tracking, track distance calculations, and progress monitoring.
- Examples: `arcproLab/mdp/track_manager.py`.
- Pattern: Singleton/Utility used within MDP functions.

**HierarchicalPolicy:**
- Purpose: Separates planning from control.
- Examples: `arcproLab/policy_stack/policies/hierarchical_policy.py`.
- Pattern: Actor-Critic with internal latent representations and separate heads.

## Entry Points

**Training Script:**
- Location: `arcproLab/scripts/train_policy.py` (via `train.sh`).
- Responsibilities: Initializes the environment and starts the RL training loop.

**Verification Script:**
- Location: `arcproLab/scripts/verify_policy.py`.
- Responsibilities: Runs inference with a trained model and visualizes metrics.

## Error Handling

**Strategy:** Exception raising and standard Python logging.

**Patterns:**
- `warnings.warn` for non-critical sim issues.
- `try-except` blocks in training loop to catch SB3-contrib specific RNN state errors.

## Cross-Cutting Concerns

**Logging:** Local TensorBoard logging via SB3.
**Validation:** `track_manager.py` performs waypoint validation.
**Metric Scaling:** Consistent 1.0x metric scale enforcement in `arcpro_env_cfg.py`.

---

*Architecture analysis: 2024-10-24*
