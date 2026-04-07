# Architecture

**Analysis Date:** 2025-04-18

## Pattern Overview

**Overall:** Manager-Based Reinforcement Learning Environment (IsaacLab 1.0+ pattern) with Gymnasium 1.0+ standards.

**Key Characteristics:**
- **Manager-Based RL**: Decoupled scene, observation, action, reward, and termination management using IsaacLab's `ManagerBasedRLEnv`.
- **Hybrid Scale Baseline (8.0x)**: The robot is scaled to 8.0x in the scene (`arcproLab/arcpro_env_cfg.py`) for visual and physical stability, while observations are normalized to 1.0x metric scale for policy compatibility.
- **4WD True Physics**: All-Wheel Drive dynamics with **20kg mass** and optimized PhysX TGS solver.
- **Vectorized Telemetry**: A standardized 12-float protocol for vehicle state and navigation intent.

## Layers

**Configuration Layer:**
- Purpose: Defines the scene, robot properties, and training hyperparameters.
- Location: `arcproLab/arcpro_env_cfg.py`, `arcproLab/arcpro_robot_cfg.py`.
- Contains: `ARCProEnvCfg`, `ARCProSceneCfg`, `ArcProRobotCfg`.
- Depends on: `isaaclab.utils.configclass`, `isaaclab.assets`, `isaaclab.sim`.

**MDP Layer (Markov Decision Process):**
- Purpose: Implements RL logic (Observations, Rewards, Events, Terminations).
- Location: `arcproLab/mdp/`.
- Contains: `observations.py` (12-float protocol with 0.125 scale normalization), `rewards.py` (speed and lateral error), `track_manager.py` (waypoint navigation).
- Used by: `arcpro_env_cfg.py`.

**Policy Layer:**
- Purpose: Neural network architectures and RL algorithms.
- Location: `arcproLab/policy_stack/`, `arcproLab/mdp/policy_wrapper.py`.
- Contains: `policies/fusion_policy.py`, `policies/hierarchical_policy.py`.
- Depends on: `stable_baselines3`, `torch`.

**Asset Layer:**
- Purpose: Storage for USD assets and navigation data.
- Location: `arcproLab/assets/`, `openStreetUSD/`.
- Contains: `F1Tenth_Metric.usd`, `no_graph_sim.usd`, `track_centerline.npy`.

## Data Flow

**Simulation Cycle:**

1. **Reset/Event**: `mdp/events.py` (not yet fully implemented for spawning) or standard IsaacLab reset.
2. **Observation Gathering**: `mdp/observations.py` calculates the 12-float telemetry vector. Note: `SPEED` and `LAT_ERR` are multiplied by `0.125` to convert from 8x sim units to 1x metric units.
3. **Policy Step**: SB3 policy computes actions based on observations.
4. **Action Application**: `ActionCfg` in `arcpro_env_cfg.py` maps actions to wheel steering and drive velocities.
5. **Physics Step**: PhysX simulates 200Hz dynamics with TGS solver.
6. **Reward Calculation**: `mdp/rewards.py` computes rewards based on current state.

**State Management:**
- Physics state managed by Isaac Sim.
- Waypoint/Navigation state managed by `TrackManager` (`arcproLab/mdp/track_manager.py`).

## Key Abstractions

**TrackManager:**
- Purpose: Vectorized waypoint tracking. Can sample waypoints directly from USD road meshes.
- Location: `arcproLab/mdp/track_manager.py`.
- Pattern: Singleton/Utility.

**12-Float Telemetry Protocol:**
- Purpose: Standardized input for the road-following policy.
- Location: `arcproLab/mdp/observations.py`.

## Entry Points

**Training Script:**
- Location: `arcproLab/scripts/train_policy.py`.
- Invoked via: `train.sh`.

**Verification Script:**
- Location: `arcproLab/scripts/verify_policy.py`.
- Responsibilities: Model inference and metric visualization.

**Asset Audit:**
- Location: `arcproLab/scripts/audit_assets.py`.
- Responsibilities: Checks USD file integrity.

## Error Handling

**Strategy:** Python exceptions for configuration errors; warning filters for simulation noise.

**Patterns:**
- NaN handling in `observations.py` and `rewards.py`.
- Fallback navigation logic in `track_manager.py` if USD sampling fails.

## Cross-Cutting Concerns

**Logging:** TensorBoard (SB3).
**Scale Normalization:** Observations are normalized from 8x to 1x in `observations.py`.
**Physics Fidelity:** 200Hz frequency with TGS solver for 20kg chassis stability.

---

*Architecture analysis: 2025-04-18*
