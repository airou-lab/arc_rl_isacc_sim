# Architecture

**Analysis Date:** 2026-05-16

## Pattern Overview

**Overall:** Hierarchical Multi-Agent Reinforcement Learning (MARL) Environment.

**Key Characteristics:**
- **Vectorized Navigation Management:** Transitioned from singleton `RoadGraph` to `RoadManager` which manages state as `(num_envs, num_agents)` tensors.
- **Hybrid Survival Logic:** Differentiates between "precision penalties" (0.15m) and "hard failures" (0.25m) to encourage recovery over simple avoidance.
- **Integrated Telemetry:** Consolidated 12-element telemetry vector for unified policy observation and real-time UI tracking.
- **Decoupled Arbitration:** Multi-agent intersection coordination handled by `SchedulerCore` in the `policy_stack`.

## Layers

**Simulation Layer:**
- Purpose: Handles physics, rendering, and USD stage management at 500Hz.
- Location: `openStreetUSD/`, `arcproLab/arcpro_env_cfg.py`.
- Contains: USD files (Metric scale), Multi-agent robot spawners.

**MDP Layer (Markov Decision Process):**
- Purpose: Defines the multi-agent observation and action spaces.
- Location: `arcproLab/mdp/`
- Contains: `observations.py`, `actions.py`, `rewards.py`, `road_manager.py`, `terminations.py`, `visual_analytics.py`.
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

**Multi-Agent Navigation & Telemetry Flow:**

1. **Mission Assignment:** `RoadManager` randomizes `turn_token` (LEFT, STRAIGHT, RIGHT) for each agent on reset.
2. **Intent Registration:** As agents approach intersections, `WorkerScheduler` (via observations) registers intent with `SchedulerCore`.
3. **Arbitration:** `SchedulerCore` checks path conflicts and time-gaps, returning a `go_signal` (1.0 or 0.0).
4. **Observation Synthesis:** `get_telemetry_vector` combines `turn_token`, `go_signal`, local velocities, and previous actions into a 12-dim tensor.
5. **Planning:** `HierarchicalPathPlanningPolicy` predicts waypoints conditioned on navigation commands and telemetry.
6. **Action:** Control head converts planned waypoints into Ackermann commands, respecting the `go_signal`.

## Key Abstractions

**RoadManager:**
- Purpose: Replaces the singleton `RoadGraph`. Manages navigation state (tokens/signals) as vectorized tensors of shape `(num_envs, num_agents)`.
- Location: `arcproLab/mdp/road_manager.py`
- Pattern: Vectorized singleton (shared instance, per-agent data).

**Telemetry Vector:**
- Purpose: Standardized 12-element observation slice used by all ARCPro policies.
- Indices: 0-2 (Nav Intent), 3 (Speed), 4 (Yaw Rate), 5-7 (Last Actions), 8-11 (Reserved/Physics).
- Location: `arcproLab/mdp/observations.py`.

**TrackManager:**
- Purpose: Handles geometry-based calculations (lateral error, heading error, boundary distances).
- Location: `arcproLab/mdp/track_manager.py`
- Optimization: Uses `.npz` caching for fast startup.

**Hybrid Boundary System:**
- Purpose: Provides a safety margin for learning.
- Logic: 
  - `boundary_penalty`: -100.0 penalty if `dist < 0.15m`.
  - `roadmark_contact`: Termination if `dist < 0.25m`.

## Entry Points

**Training Entry Point:**
- Location: `arcproLab/scripts/train_policy.py`
- Responsibilities: Initializes Isaac Sim, builds vectorized environments, and starts training.

**Telemetry Relaunch:**
- Location: `relaunch_with_telemetry.sh`
- Responsibilities: Resumes training with enhanced telemetry logging and visual analytics.

---

*Architecture analysis: 2026-05-16*
