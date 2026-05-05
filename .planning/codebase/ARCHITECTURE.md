# Architecture

**Analysis Date:** 2024-05-18

## Pattern Overview

**Overall:** Hierarchical Reinforcement Learning with Manager-Based Simulation Environment.

**Key Characteristics:**
- **Manager-Based RL Environment:** Built on NVIDIA Isaac Lab, using configurations to define scenes, observations, actions, rewards, and terminations.
- **Hierarchical Policy:** Separates high-level path planning (waypoints) from low-level control (steering/throttle).
- **Kinematic Anchors:** Inductive bias for the policy using curved paths derived from navigation intent and vehicle state.
- **Modular MDP:** Component-based MDP (Markov Decision Process) implementation with dedicated classes for track management and road graph logic.

## Layers

**Simulation Layer (Isaac Sim/Lab):**
- Purpose: High-fidelity physics and visual simulation.
- Location: Native NVIDIA Omniverse / Isaac Lab libraries.
- Contains: PhysX engine, USD stage management, sensor simulation (TiledCamera, ContactSensor).
- Depends on: NVIDIA Driver, CUDA.
- Used by: Environment Layer.

**Environment Layer:**
- Purpose: Bridges Isaac Sim to Gymnasium-compatible RL interfaces.
- Location: `arcproLab/arcpro_env_cfg.py`, `arcproLab/arcpro_robot_cfg.py`
- Contains: Scene configuration, asset definitions, MDP manager registrations.
- Depends on: Isaac Lab, MDP Layer.
- Used by: Execution Layer.

**MDP Layer:**
- Purpose: Implements the RL logic (observations, rewards, terminations).
- Location: `arcproLab/mdp/`
- Contains: `track_manager.py` (road-relative state), `road_graph.py` (navigation logic), `rewards.py`, `observations.py`.
- Depends on: Isaac Lab, PyTorch.
- Used by: Environment Layer.

**Policy Layer:**
- Purpose: The autonomous agent's "brain".
- Location: `arcproLab/policy_stack/policies/`
- Contains: `hierarchical_policy.py` (HPPP), `fusion_policy.py` (Visual + Telemetry feature extraction).
- Depends on: Stable Baselines3 (SB3), sb3-contrib (RecurrentPPO), PyTorch.
- Used by: Execution Layer.

**Execution Layer:**
- Purpose: Training and verification entry points.
- Location: `arcproLab/scripts/`
- Contains: `train_policy.py`, `verify_policy.py`, `verify_drive.py`.
- Depends on: Environment Layer, Policy Layer.

## Data Flow

**Training Loop:**

1. `train_policy.py` initializes the `ManagerBasedRLEnv` using `ARCProEnvCfg`.
2. Environment is wrapped by `WaypointTrackingWrapper` for auxiliary losses.
3. `HPPPDirectBridge` translates Isaac Lab observations/actions to HPPP/SB3 format.
4. `RecurrentPPO` (from `sb3-contrib`) collects rollouts:
    - `HierarchicalPathPlanningPolicy.forward()` computes waypoints and actions.
    - Isaac Sim steps physics at 500Hz (decimated to 20Hz for control).
5. Rewards and terminations are calculated by managers in `arcproLab/mdp/`.
6. Policy is updated using PPO loss + auxiliary waypoint loss.

**State Management:**
- **Simulation State:** Managed by Isaac Sim (PhysX).
- **Agent Memory:** Managed by LSTM hidden states in `HierarchicalPathPlanningPolicy`.
- **Navigation State:** Managed by `RoadGraph` (Turn Tokens) and `TrackManager` (Lateral/Heading errors).

## Key Abstractions

**TrackManager:**
- Purpose: Provides geometric context relative to the track centerline and boundaries.
- Examples: `arcproLab/mdp/track_manager.py`
- Pattern: Singleton/Cached USD parser.

**RoadGraph:**
- Purpose: Handles high-level navigation intent (intersections).
- Examples: `arcproLab/mdp/road_graph.py`
- Pattern: USD Attribute parser with Turn Token logic.

**HierarchicalPathPlanningPolicy (HPPP):**
- Purpose: Decouples path planning from control.
- Examples: `arcproLab/policy_stack/policies/hierarchical_policy.py`
- Pattern: Hierarchical Actor-Critic with Kinematic Anchors.

## Entry Points

**Training Script:**
- Location: `arcproLab/scripts/train_policy.py`
- Triggers: Manual execution via `python arcproLab/scripts/train_policy.py`.
- Responsibilities: CLI parsing, AppLauncher initialization, environment creation, policy setup, training loop.

**Verification Script:**
- Location: `arcproLab/scripts/verify_policy.py`
- Triggers: Manual execution.
- Responsibilities: Load trained model, run evaluation episodes, visualize waypoint predictions.

## Error Handling

**Strategy:** Fail-fast on configuration errors; graceful resets on simulation instability.

**Patterns:**
- **Termination Managers:** Detect "bad" states (crashes, driving blind, falling) and trigger environment resets via `arcproLab/mdp/terminations.py`.
- **Validation Checks:** Scripts like `arcproLab/scripts/audit_assets.py` verify USD integrity before training.

## Cross-Cutting Concerns

**Logging:** Tensorboard via SB3, custom CSV/text logging for curriculum tracking (`training_curriculum.log`).
**Validation:** Extensive use of `VisualizationMarkers` in `TrackManager` for real-time debugging in the Isaac Sim GUI.
**Authentication:** Not applicable (Local simulation).

---

*Architecture analysis: 2024-05-18*
