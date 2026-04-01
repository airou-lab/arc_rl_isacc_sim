# Architecture

**Analysis Date:** 2024-10-24

## Pattern Overview

**Overall:** Manager-Based Reinforcement Learning Environment (IsaacLab pattern) with Hierarchical Planning.

**Key Characteristics:**
- **Hierarchical Policy Architecture**: A two-stage policy that separates path planning (waypoint prediction) from low-level Ackermann control.
- **True Physics Integration**: Robot operates at 0.5x metric scale with realistic mass (3.5kg) and front-wheel drive (FWD) dynamics.
- **Vectorized Telemetry**: A standardized 12-float protocol for vehicle state and navigation intent.

## Layers

**Configuration Layer:**
- Purpose: Defines the scene, robot properties (0.5x scale), and training hyperparameters.
- Location: `arcproLab/arcpro_env_cfg.py`, `arcproLab/arcpro_robot_cfg.py`.
- Contains: IsaacLab configuration classes.
- Depends on: `isaaclab.utils.configclass`.

**MDP Layer (Markov Decision Process):**
- Purpose: Implements the RL interface (Observations, Rewards, etc.).
- Location: `arcproLab/mdp/`.
- Contains: `observations.py` (12-float protocol), `rewards.py` (Gaussian-weighted), `terminations.py`, `events.py`, `track_manager.py`.
- Used by: `arcpro_env_cfg.py`.

**Policy Layer:**
- Purpose: Implements neural network architectures and RL algorithms.
- Location: `arcproLab/policy_stack/`.
- Contains: `policies/hierarchical_policy.py`, `policies/fusion_policy.py`.
- Depends on: `stable_baselines3`, `sb3_contrib`, `torch`.

**Asset Layer:**
- Purpose: Stores USD assets and pre-computed track data.
- Location: `arcproLab/assets/`, `openStreetUSD/`.
- Contains: `F1Tenth_Metric.usd` (0.5x scale), `no_graph_sim_final.usd` (stable track), `track_centerline.npy` (absolute waypoints).

## Data Flow

**Training Loop:**

1. **Environment Initialization**: `arcpro_env_cfg.py` loads the track and robot (0.5x scale).
2. **Observation Gathering**: `mdp/observations.py` computes the **12-float telemetry vector** and vision input.
3. **Policy Inference**: `policy_stack/policies/hierarchical_policy.py` predicts waypoint deviations from kinematic anchors and converts them to Ackermann commands.
4. **Action Application**: Commands (steer, throttle) are sent to the **Front-Wheel Drive** actuators (`Joint_Drive_FL`, `Joint_Drive_FR`).
5. **Reward & Termination**: `mdp/rewards.py` evaluates performance using Gaussian-weighted lateral error rewards.

**State Management:**
- Handled by Isaac Sim's physics state and managed via IsaacLab's `ManagerBasedRLEnv`.

## Key Abstractions

**TrackManager:**
- Purpose: Handles **absolute waypoint alignment**, track distance calculations, and curvature (Kappa) estimation.
- Examples: `arcproLab/mdp/track_manager.py`.
- Pattern: Singleton/Utility used within MDP functions.

**HierarchicalPolicy:**
- Purpose: Separates planning from control. Uses a 12-float telemetry vector to compute kinematic anchors.
- Examples: `arcproLab/policy_stack/policies/hierarchical_policy.py`.
- Pattern: Actor-Critic with internal latent representations and separate planning/control heads.

## Entry Points

**Training Script:**
- Location: `arcproLab/scripts/train_policy.py` (via `train.sh`).
- Responsibilities: Initializes the environment and starts the RL training loop.

**Verification Script:**
- Location: `arcproLab/scripts/verify_policy.py`.
- Responsibilities: Runs inference with a trained model and visualizes metrics via the 12-float protocol.

## Error Handling

**Strategy:** Exception raising and standard Python logging.

**Patterns:**
- `warnings.warn` for non-critical sim issues.
- `try-except` blocks in training loop to catch SB3-contrib specific RNN state errors.

## Cross-Cutting Concerns

**Logging:** Local TensorBoard logging via SB3.
**Validation:** `track_manager.py` performs absolute waypoint validation.
**Metric Scaling:** Consistent **0.5x robot scale** enforcement across assets and configuration.

---

*Architecture analysis: 2024-10-24*
