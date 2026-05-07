# Codebase Structure

**Analysis Date:** 2024-11-20

## Directory Layout

```
arc_rl_isacc_sim/
├── arcproLab/              # Main source code
│   ├── assets/             # Robot and environment assets
│   ├── mdp/                # MDP components (Observations, Rewards, etc.)
│   ├── models/             # Saved policy models (.pth)
│   ├── policy_stack/       # RL algorithm and policy implementations
│   └── scripts/            # Training, verification, and utility scripts
├── docs/                   # Project documentation
├── logs/                   # Training logs and telemetry
├── openStreetUSD/          # Simulation scene files (.usda)
├── tensorboard_view_docker/# Visualization infrastructure
└── trash/                  # Deprecated or temporary files
```

## Directory Purposes

**arcproLab/mdp/:**
- Purpose: Defines the Markov Decision Process interface.
- Contains: Action/Observation/Reward/Termination terms.
- Key files: `track_manager.py`, `road_graph.py`, `observations.py`.

**arcproLab/policy_stack/:**
- Purpose: Custom RL framework and policy architectures.
- Contains: Agents, policies, wrappers, and tests.
- Key files: `train_policy_ros2.py`, `isaac_direct_env.py`.

**arcproLab/scripts/:**
- Purpose: Operational scripts for the project.
- Contains: Training entry points and extensive verification tools.
- Key files: `train_policy.py`, `verify_policy.py`, `verify_metric.py`.

**openStreetUSD/:**
- Purpose: 3D assets and scene definitions for Isaac Sim.
- Contains: USD/USDA files.
- Key files: `no_graph_sim_clean_1x_flattened.usda`.

## Key File Locations

**Entry Points:**
- `arcproLab/scripts/train_policy.py`: Primary training script.
- `arcproLab/scripts/verify_policy.py`: Policy evaluation script.

**Configuration:**
- `arcproLab/arcpro_env_cfg.py`: Environment configuration.
- `arcproLab/arcpro_robot_cfg.py`: Robot asset and actuator configuration.

**Core Logic:**
- `arcproLab/mdp/track_manager.py`: Handles track geometry and boundaries.
- `arcproLab/mdp/road_graph.py`: Manages road connectivity and intersections.

**Testing:**
- `arcproLab/policy_stack/tests/`: Unit tests for policy components.
- `arcproLab/scripts/verify_*.py`: Simulation-based functional tests.

## Naming Conventions

**Files:**
- Snake_case: `track_manager.py`, `train_policy.py`.

**Directories:**
- Snake_case or CamelCase (e.g., `arcproLab`, `openStreetUSD`).

## Where to Add New Code

**New Feature (e.g., Obstacle Avoidance):**
- Primary code: `arcproLab/mdp/observations.py`, `arcproLab/mdp/rewards.py`.
- Tests: `arcproLab/policy_stack/tests/` (if unit testable) or `arcproLab/scripts/verify_obstacles.py`.

**New Component/Module:**
- Implementation: `arcproLab/mdp/` for simulation-related logic, `arcproLab/policy_stack/` for learning-related logic.

**Utilities:**
- Shared helpers: `arcproLab/mdp/` (for geometry/sim) or `arcproLab/scripts/` (for CLI tools).

## Special Directories

**trash/:**
- Purpose: Contains deprecated scripts and temporary debug files to keep the main directories clean.
- Committed: Yes.

**logs/:**
- Purpose: Contains Tensorboard event files and training telemetry.
- Committed: No (typically ignored by `.gitignore`).

---

*Structure analysis: 2024-11-20*
