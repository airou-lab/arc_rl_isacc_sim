# Architecture

**Analysis Date:** 2025-04-05

## Pattern Overview

**Overall:** Manager-Based Reinforcement Learning Environment (IsaacLab 1.0+ pattern) with Gymnasium 1.0+ standards.

**Key Characteristics:**
- **Hierarchical Policy Architecture**: A two-stage policy that separates path planning (waypoint prediction) from low-level Ackermann/4WD control.
- **True Physics Integration (AWD)**: All-Wheel Drive dynamics with **20kg mass** and optimized PhysX TGS solver.
- **Hybrid Scale Mode**: Track visuals maintained at 1.0x (original) with robot scaled to 8.0x in scene to match visual proportions, while internal control logic remains metric-aligned.
- **Vectorized Telemetry**: A standardized 12-float protocol for vehicle state and navigation intent.

## Layers

**Configuration Layer:**
- Purpose: Defines the scene, robot properties, and training hyperparameters using IsaacLab's manager-based system.
- Location: `arcproLab/arcpro_env_cfg.py`, `arcproLab/arcpro_robot_cfg.py`.
- Contains: IsaacLab configuration classes with optimized Physics (200Hz, TGS).
- Depends on: `isaaclab.utils.configclass`, `gymnasium`.

**MDP Layer (Markov Decision Process):**
- Purpose: Implements the RL interface (Observations, Rewards, etc.) according to Gymnasium 1.0+ standards.
- Location: `arcproLab/mdp/`.
- Contains: `observations.py` (12-float protocol), `rewards.py` (Gaussian-weighted), `terminations.py`, `events.py`, `track_manager.py`.
- Used by: `arcpro_env_cfg.py`.

**Policy Layer:**
- Purpose: Implements neural network architectures and RL algorithms.
- Location: `arcproLab/policy_stack/`, `arcproLab/mdp/policy_wrapper.py`.
- Contains: `policies/hierarchical_policy.py`, `policies/fusion_policy.py`, `policy_wrapper.py` (SB3/Inference wrapper).
- Depends on: `stable_baselines3`, `sb3_contrib`, `torch`.

**Asset Layer:**
- Purpose: Stores USD assets and pre-computed track data.
- Location: `arcproLab/assets/`, `openStreetUSD/`.
- Contains: `F1Tenth_Metric.usd`, `no_graph_sim.usd` (Repaired/Hardened track), `track_centerline.npy`.

## Data Flow

**Training Loop:**

1. **Environment Initialization**: `arcpro_env_cfg.py` loads the track and robot.
2. **Environment Wrapping**: `Sb3VecEnvWrapper` (from `isaaclab_rl.sb3`) wraps the IsaacLab environment for SB3 compatibility.
3. **Observation Gathering**: `mdp/observations.py` computes the **12-float telemetry vector** and vision input.
4. **Policy Inference**: SB3 policy predicts waypoint deviations and converts them to Ackermann/Velocity commands.
5. **Action Application**: Commands (steer, throttle) are sent to the **4WD** actuators (`Joint_Drive_.*`).
6. **Reward & Termination**: `mdp/rewards.py` evaluates performance using Gaussian-weighted lateral error rewards.

**State Management:**
- Handled by Isaac Sim's physics state and managed via IsaacLab's `ManagerBasedRLEnv`.

## Key Abstractions

**TrackManager:**
- Purpose: Handles **absolute waypoint alignment**, track distance calculations, and curvature (Kappa) estimation.
- Examples: `arcproLab/mdp/track_manager.py`.
- Pattern: Singleton/Utility used within MDP functions.

**Sb3VecEnvWrapper:**
- Purpose: Modernized wrapper for connecting IsaacLab environments to Stable Baselines3.
- Location: `isaaclab_rl.sb3`.

**PolicyWrapper:**
- Purpose: Standardized interface for loading and running inference on trained models (ResNet/SB3).
- Examples: `arcproLab/mdp/policy_wrapper.py`.

## Entry Points

**Training Script:**
- Location: `arcproLab/scripts/train_policy.py` (via `train.sh`).
- Responsibilities: Initializes the environment (Gymnasium 1.0+), wraps for SB3, and starts the training loop.

**Verification Script:**
- Location: `arcproLab/scripts/verify_policy.py`, `arcproLab/verify.py`.
- Responsibilities: Runs inference with a trained model and visualizes metrics via the 12-float protocol.

## Error Handling

**Strategy:** Exception raising and standard Python logging with specific Gymnasium warning filters.

**Patterns:**
- `warnings.filterwarnings` to silence legacy gym/sim noise.
- `try-except` blocks for USD asset loading and SB3 RNN state errors.

## Cross-Cutting Concerns

**Logging:** Local TensorBoard logging via SB3; visual analytics in `visual_analytics.py`.
**Validation:** `track_manager.py` performs absolute waypoint validation.
**Physics Optimization:** High-frequency (200Hz) simulation with TGS solver for stability of the 20kg AWD chassis.
**Asset Integrity:** `repair_usd_references.py` ensures track visuals load without unresolved reference errors.

---

*Architecture analysis: 2025-04-05*
