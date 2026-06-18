# Architecture

**Analysis Date:** 2024-06-16

## Pattern Overview

**Overall:** Manager-Based Reinforcement Learning Framework (IsaacLab) bridged with Multi-Agent (SKRL) and continuous monitoring.

**Key Characteristics:**
- **Vectorized Environments**: Isaac Sim physics processes all agents and environments in parallel tensors (GPU).
- **Decoupled Management**: Domain specifics (Road, Track, Policies) are isolated into vectorized managers.
- **Auto-Monitoring**: An independent watchdog service monitors training metrics to halt collapsing sessions automatically.

## Layers

**Environment Layer:**
- Purpose: Defines the rules, state, and physics constraints of the simulation.
- Location: `arcproLab/arcpro_env_cfg.py`, `arcproLab/mdp/`
- Contains: MDP managers (rewards, terminations, observations, actions).
- Depends on: Isaac Sim, Omniverse USD APIs.
- Used by: RL policies during training.

**Policy & Agent Layer:**
- Purpose: Handles the decision-making network, hierarchy, and neural outputs.
- Location: `arcproLab/policy_stack/`
- Contains: SKRL integration, hierarchical policies, agent wrappers.
- Depends on: `arcproLab/mdp` state vectors, PyTorch.
- Used by: Inference nodes, ROS2 bridges.

**Monitoring Layer:**
- Purpose: Provides safety checks and diagnoses training divergence.
- Location: `arcproLab/scripts/watchdog.py`
- Contains: Continuous regex log parsers and subprocess control.
- Depends on: Live log outputs in `logs/`.
- Used by: Training launch scripts.

## Data Flow

**Multi-Agent State Pipeline (Phase 16):**

1. Isaac Sim updates vehicle physics.
2. `arcproLab/mdp/road_manager.py` (Vectorized) computes per-agent navigation intents and intersection states, replacing the legacy `RoadGraph` singleton.
3. `arcproLab/mdp/track_manager.py` syncs nearest waypoints and local offsets.
4. Observations and rewards are calculated via `arcproLab/mdp/rewards.py` using tracked index vectors.
5. SKRL processes the tensor batch and outputs new motor actions.

**State Management:**
- Maintained as `(num_envs, num_agents)` PyTorch tensors on the device (`cuda:0`).
- State caching (like previous actions) occurs inside `env.extras` dictionary in IsaacLab.

## Key Abstractions

**`RoadManager`:**
- Purpose: Manages high-level routing, turn tokens, and multi-agent coordination.
- Examples: `arcproLab/mdp/road_manager.py`
- Pattern: Vectorized singleton (returned via `get_road_manager()`).

**`TrackManager`:**
- Purpose: Handles geometric distance mapping, waypoints, and centerline offsets.
- Examples: `arcproLab/mdp/track_manager.py`
- Pattern: Vectorized singleton with caching (`last_indices`) to prevent expensive nearest-neighbor recalculations every step.

## Entry Points

**Training Pipeline:**
- Location: `arcproLab/scripts/train_policy.py`
- Triggers: Shell execution (`run_train_bg.sh`).
- Responsibilities: Initializes IsaacLab, registers environments, and launches the RL loop.

**Watchdog Monitor:**
- Location: `arcproLab/scripts/watchdog.py`
- Triggers: Background execution alongside training.
- Responsibilities: Parses logs for scientific notation standard deviation and FPS drops, issuing `tmux kill-session` if divergence or stagnation is detected.

## Error Handling

**Strategy:** Fail-fast at the training/monitoring level, continuous gradients at the reward level.

**Patterns:**
- **Rewards**: Continuous penalty scaling (`lateral_error_reward`, `jerk_penalty`) prevents sudden value drop-offs, aiding stabilization.
- **Monitoring**: `watchdog.py` generates `WATCHDOG_DIAGNOSIS.md` upon failure, ensuring failures are documented before killing the process.

## Cross-Cutting Concerns

**Logging:** Standard output routed to `logs/` directory, parsed dynamically.
**Vectorization:** Every component in `arcproLab/mdp/` must accept and return tensors shaped for `(num_envs, num_agents)` to conform to the Phase 16 multi-agent transition.

---

*Architecture analysis: 2024-06-16*