# Codebase Structure

**Analysis Date:** 2024-06-16

## Directory Layout

```
[project-root]/
├── .planning/                  # Project management, phase tracking, and mapped codebase docs
├── arcproLab/                  # Main IsaacLab environments and modules
│   ├── assets/                 # Robot and environment USD assets
│   ├── mdp/                    # Manager-based RL environment components (actions, rewards, etc.)
│   ├── policy_stack/           # SKRL policies, baselines, and multi-agent coordination
│   ├── scripts/                # Utility scripts, watchdogs, auditing, and training entry points
│   └── models/                 # Saved models and checkpoints
├── openStreetUSD/              # Flattened and layered USD tracks for Isaac Sim
└── tensorboard_view_docker/    # Local Tensorboard deployment configuration
```

## Directory Purposes

**`arcproLab/mdp/`:**
- Purpose: Contains the core reinforcement learning definitions for IsaacLab.
- Contains: Managers for observations, actions, rewards, terminations, and events.
- Key files: `road_manager.py`, `track_manager.py`, `rewards.py`, `spawner.py`

**`arcproLab/policy_stack/`:**
- Purpose: Handles the multi-agent reinforcement learning (MARL) policies and bridging logic.
- Contains: SKRL neural network definitions, geometry calibrators, inference servers, and custom policy wrappers.
- Key files: `policies/hierarchical_policy.py`, `agent/agent_env_wrapper.py`, `train_policy_ros2.py`

**`arcproLab/scripts/`:**
- Purpose: Operational, debugging, and training scripts.
- Contains: Watchdogs, environment audits, training loops, verification tools.
- Key files: `watchdog.py`, `train_policy.py`, `audit_vision.py`, `test_terminations.py`

## Key File Locations

**Entry Points:**
- `arcproLab/scripts/train_policy.py`: Primary single-agent training script.
- `arcproLab/scripts/watchdog.py`: Monitoring and auto-recovery script for training runs.

**Configuration:**
- `arcproLab/arcpro_env_cfg.py`: Primary IsaacLab environment configuration.
- `arcproLab/arcpro_robot_cfg.py`: Configuration for the autonomous vehicle model and articulations.

**Core Logic:**
- `arcproLab/mdp/road_manager.py`: Vectorized road graph handling for multi-agent scene context, replacing the legacy `RoadGraph` singleton.
- `arcproLab/mdp/track_manager.py`: Handles caching of waypoints and centerline distance logic.
- `arcproLab/mdp/rewards.py`: Contains dense shaping rewards, termination penalties, and lateral offset continuous gradients.

**Testing:**
- `arcproLab/scripts/test_terminations.py`: Tests the reset and termination logic in the environment.
- `arcproLab/scripts/deep_verify.py`: Performs a comprehensive check of all subsystems.

## Naming Conventions

**Files:**
- Snake Case: `road_manager.py`, `test_terminations.py`

**Directories:**
- Snake Case: `policy_stack`, `logs` (with the exception of `arcproLab` and `openStreetUSD` which use camelCase variants).

## Where to Add New Code

**New Feature (MDP or Core RL Logic):**
- Primary code: `arcproLab/mdp/`
- Register in: `arcproLab/arcpro_env_cfg.py`

**New Component/Module (MARL Policies):**
- Implementation: `arcproLab/policy_stack/policies/` or `arcproLab/policy_stack/agent/`

**Utilities:**
- Shared helpers: `arcproLab/scripts/`

## Special Directories

**`.planning/`:**
- Purpose: GSD system data including plans, memories, and state.
- Generated: Yes (managed by system/user)
- Committed: Yes

**`logs/`:**
- Purpose: Holds tensorboard logs, check-points, and the `WATCHDOG_DIAGNOSIS.md`.
- Generated: Yes
- Committed: No (generally ignored)

---

*Structure analysis: 2024-06-16*