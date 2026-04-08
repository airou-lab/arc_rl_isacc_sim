# Codebase Structure

**Analysis Date:** 2025-04-28

## Directory Layout

```
[project-root]/
├── arcproLab/          # Core RL logic and environment
│   ├── assets/         # Simulation assets (F1Tenth_Metric.usd)
│   ├── mdp/            # RL interface logic (Obs, Rew, Events)
│   │   ├── events.py   # Spawning and domain randomization
│   │   ├── observations.py # 12-float telemetry and vision input
│   │   ├── rewards.py  # Gaussian-weighted reward functions
│   │   ├── terminations.py # Episode termination conditions
│   │   └── track_manager.py # Absolute waypoint management
│   ├── models/         # Trained model checkpoints (.pth)
│   ├── scripts/        # Training, verification, and audit tools
│   │   ├── audit_assets.py # USD integrity checks
│   │   ├── train_policy.py # Main SB3 training script
│   │   ├── verify_metric.py # Physical performance audit
│   │   ├── verify_policy.py # Model evaluation
│   │   └── verify_spawn.py # Spawn and camera sanity checks
│   ├── arcpro_env_cfg.py # Main environment configuration
│   └── arcpro_robot_cfg.py # Main robot articulation configuration
├── .planning/          # GSD Project Planning and Mapping
│   ├── codebase/       # Architecture and convention documents
│   ├── phases/         # Current development phases (e.g., Phase 09)
│   └── todos/          # Stabilization and hygiene tasks
├── openStreetUSD/      # Map assets (no_graph_sim.usd)
├── tests/              # Pytest logic tests (test_track_manager.py)
├── trash/              # Legacy tools and experimental scripts
├── train.sh            # Training launcher
├── verify_sim.sh       # Headless simulation verification
└── run_gui_verify.sh   # GUI-based model verification
```

## Directory Purposes

**arcproLab/mdp/:**
- Purpose: Bridge between Isaac Sim physics and the RL policy.
- Contains: Logic for translating raw sensor data into the **12-float telemetry vector**.
- Key files: `observations.py`, `track_manager.py`, `events.py`.

**arcproLab/scripts/:**
- Purpose: Standalone execution scripts for common tasks.
- Key files: `verify_spawn.py` (primary verification for Phase 09), `train_policy.py`.

**openStreetUSD/:**
- Purpose: Storage for world geometry and road networks.
- Contains: `no_graph_sim.usd` (The primary track used in `arcpro_env_cfg.py`).

**.planning/:**
- Purpose: Management of project roadmap, phases, and task-driven stabilization.
- Contains: `ROADMAP.md`, `STATE.md`, and focus-specific todos in `todos/`.

## Key File Locations

**Entry Points:**
- `arcproLab/scripts/train_policy.py`: Main SB3 training script.
- `arcproLab/scripts/verify_spawn.py`: Sanity check for robot/track alignment and cameras.
- `run_gui_verify.sh`: Script to launch GUI verification of trained policies.

**Configuration:**
- `arcproLab/arcpro_env_cfg.py`: Environment and Scene setup (defines 8x robot scale, camera flags).
- `arcproLab/arcpro_robot_cfg.py`: Robot physical parameters (20kg mass, 4WD).

**Core Logic:**
- `arcproLab/mdp/track_manager.py`: Waypoint logic and USD mesh sampling.
- `arcproLab/mdp/events.py`: Logic for resetting the robot to the lane.

**Testing:**
- `tests/test_track_manager.py`: Unit tests for waypoint logic.
- `verify_sim.sh`: Automated simulation performance verification.

## Naming Conventions

**Files:**
- snake_case for scripts and modules: `arcpro_env_cfg.py`.

**Directories:**
- camelCase or snake_case: `arcproLab/`, `openStreetUSD/`.

## Where to Add New Code

**Phase 09 Stabilization Tasks:**
- New verification logic: `arcproLab/scripts/`.
- New event logic (Reset/Randomization): `arcproLab/mdp/events.py`.
- Task tracking: `.planning/todos/`.

**New Features (MDP):**
- Observation/Reward logic: `arcproLab/mdp/`.
- Unit tests: `tests/`.

**Utilities:**
- Simulation-related: `arcproLab/mdp/`.
- Deployment-related: `arcproLab/scripts/`.

## Special Directories

**trash/tools/:**
- Purpose: Legacy maintenance scripts; contains `bake_*`, `measure_*`, and `audit_*` tools for USD files.
- Committed: Yes.

---

*Structure analysis: 2025-04-28*
