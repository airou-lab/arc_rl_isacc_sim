# Architecture

**Analysis Date:** 2026-05-11

## Pattern Overview

**Overall:** Hierarchical Multi-Agent Reinforcement Learning (MARL) Environment.

**Key Characteristics:**
- **Vectorized Navigation Management:** Transitioned from singleton `RoadGraph` to `RoadManager` which manages state as `(num_envs, num_agents)` tensors.
- **Hybrid Survival Logic:** Differentiates between "precision penalties" (0.15m) and "hard failures" (0.25m) to encourage recovery over simple avoidance.
- **Decoupled Arbitration:** Multi-agent intersection coordination handled by a standalone `SchedulerCore` separated from the physics environment.

## Layers

**Simulation Layer:**
- Purpose: Handles physics, rendering, and USD stage management at 500Hz.
- Location: `openStreetUSD/`, `arcproLab/arcpro_env_cfg.py`.
- Contains: USD files (Metric scale), Multi-agent robot spawners.

**MDP Layer (Markov Decision Process):**
- Purpose: Defines the multi-agent observation and action spaces.
- Location: `arcproLab/mdp/`
- Contains: `observations.py`, `actions.py`, `rewards.py`, `road_manager.py`, `terminations.py`.
- Depends on: Simulation Layer, `TrackManager`.

**Arbitration Layer (MARL Coordination):**
- Purpose: Resolves conflicts at intersections using First-Come-First-Served (FCFS) logic.
- Location: `arcproLab/policy_stack/agent/`
- Contains: `scheduler_core.py`, `worker_scheduler.py`, `intersection_graph.py`.

**Policy Layer:**
- Purpose: Neural network architecture and training logic.
- Location: `arcproLab/policy_stack/`
- Contains: `hierarchical_policy.py`, `fusion_policy.py`.

## Data Flow

**Multi-Agent Navigation Flow:**

1. **Mission Assignment:** `RoadManager` randomizes `turn_token` (LEFT, STRAIGHT, RIGHT) for each agent on reset.
2. **Intent Registration:** As agents approach intersections, `WorkerScheduler` (via observations) registers intent with `SchedulerCore`.
3. **Arbitration:** `SchedulerCore` checks path conflicts and time-gaps, returning a `go_signal` (1.0 or 0.0).
4. **Observation:** Agents receive a 12-dim telemetry vector including their specific `turn_token` and `go_signal` fetched from `RoadManager`.
5. **Planning:** `HierarchicalPathPlanningPolicy` predicts waypoints conditioned on navigation commands.
6. **Action:** Control head converts planned waypoints into Ackermann commands, respecting the `go_signal`.

## Key Abstractions

**RoadManager:**
- Purpose: Replaces the singleton `RoadGraph`. Manages navigation state (tokens/signals) as vectorized tensors of shape `(num_envs, num_agents)`.
- Location: `arcproLab/mdp/road_manager.py`
- Pattern: Module-level global singleton access via `get_road_manager()`, but internal data is vectorized across environments and agents.

**TrackManager:**
- Purpose: Handles geometry-based calculations (lateral error, heading error, boundary distances).
- Location: `arcproLab/mdp/track_manager.py`
- Optimization: Uses `.npz` caching for fast startup.

**Hybrid Boundary System:**
- Purpose: Provides a safety margin for learning.
- Logic: 
  - `boundary_penalty`: -100.0 penalty if `dist < 0.15m`.
  - `roadmark_contact`: Termination if `dist < 0.25m`.
- Goal: Allows the agent to "touch" the hazard zone and feel the penalty before being forced to reset, enabling the learning of recovery behaviors.

## Entry Points

**Training Entry Point:**
- Location: `arcproLab/scripts/train_policy.py`
- Responsibilities: Initializes Isaac Sim, builds vectorized environments, and starts MARL training via Recurrent PPO.

---

*Architecture analysis: 2026-05-11*
