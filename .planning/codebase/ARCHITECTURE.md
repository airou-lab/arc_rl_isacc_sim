# Architecture

**Analysis Date: 2026-05-11**

## Pattern Overview

**Overall:** Hierarchical Manager-Based Reinforcement Learning Environment (Isaac Lab pattern)

**Key Characteristics:**
- **Hierarchical Planning:** Decouples path planning (waypoint prediction) from low-level control (steering/throttle).
- **Multi-Modal Fusion:** Fuses HD visual streams (Adaptive CNN) with precise 12-dim physics telemetry.
- **1.0x Metric Fidelity:** Reverted from 8x toy scale to 1.0x metric scale for real-world transferability.

## Layers

**Simulation Layer:**
- Purpose: Handles physics, rendering, and USD stage management at 500Hz.
- Location: `openStreetUSD/` (assets), `arcproLab/arcpro_env_cfg.py` (scene config).
- Contains: USD files (Metric scale), HD Camera (640x360 or 960x600).
- Depends on: NVIDIA Isaac Sim / Isaac Lab.

**MDP Layer (Markov Decision Process):**
- Purpose: Defines how the agent interacts with the simulation.
- Location: `arcproLab/mdp/`
- Contains: `actions.py`, `observations.py`, `rewards.py`, `terminations.py`, `events.py`.
- Depends on: Simulation Layer, `TrackManager`.

**Policy Layer:**
- Purpose: Implements the neural networks and RL algorithms.
- Location: `arcproLab/policy_stack/`
- Contains:
  - `FusionFeaturesExtractor`: Fuses CNN visual features with telemetry via LayerNorm.
  - `HierarchicalPathPlanningPolicy`: Recurrent Actor-Critic with internal waypoint prediction head.
- Depends on: MDP Layer.

## Data Flow

**Hierarchical RL Loop:**

1. **Step:** `isaaclab` triggers a simulation step (500Hz internal, 50Hz control).
2. **Observe:** `observations.py` extracts HD RGB image and 12-dim telemetry.
3. **Fuse:** `FusionFeaturesExtractor` processes visual data through an Adaptive CNN and concatenates with telemetry.
4. **Plan:** `HierarchicalPathPlanningPolicy` (Actor) predicts waypoint deviations from kinematic anchors.
5. **Act:** Control head converts latent memory + planned waypoints into Ackermann commands.
6. **Apply:** `actions.py` applies torques/velocities to the robot joints.
7. **Reward:** `rewards.py` calculates momentum, precision, and smoothness rewards.

## Key Abstractions

**TrackManager:**
- Purpose: Centralized access to track geometry and boundaries for distance-based rewards and resets.
- Pattern: Singleton utility injected into MDP terms.

**Hierarchical Brain:**
- Purpose: Inductive bias toward path-following by predicting deviations from curved kinematic anchors.
- Pattern: Actor-Critic with Auxiliary Planning Head.

## Entry Points

**Training Script:**
- Location: `arcproLab/scripts/train_policy.py`
- Responsibilities: Launches Isaac Sim, initializes vectorized envs, starts Recurrent PPO training.

**Verification Script:**
- Location: `arcproLab/scripts/verify_policy.py`
- Responsibilities: Evaluates trained models in high-fidelity simulation with visual telemetry.

---

*Architecture analysis: 2026-05-11*
