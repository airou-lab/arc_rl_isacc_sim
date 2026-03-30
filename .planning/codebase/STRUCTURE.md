# Codebase Structure

**Analysis Date:** 2024-10-31

## Directory Layout

```
arcpro_rl_isaac_sim/
├── arcproLab/          # Core Python logic and RL definitions
│   ├── assets/         # Simulation assets (USD files for robots)
│   ├── mdp/            # Markov Decision Process logic (rewards, obs, terms)
│   ├── models/         # Pre-trained policy models (.pth)
│   ├── scripts/        # Training and verification entry points
│   └── *_cfg.py        # Environment and Robot configuration files
├── docs/               # Technical documentation and project plans
├── openStreetUSD/      # Environment simulation scenes (.usd)
├── tests/              # Unit and integration tests
├── trash/              # Legacy tools and experimental scripts
├── .planning/          # GSD planning and codebase analysis docs
└── *.sh                # Shell script shortcuts for training and verification
```

## Directory Purposes

**arcproLab/:**
- Purpose: Main application package containing configuration and RL logic.
- Contains: Configuration classes, scripts, and MDP modules.
- Key files: `arcpro_env_cfg.py`, `arcpro_robot_cfg.py`.

**arcproLab/mdp/:**
- Purpose: Logic for state observations, rewards, and episode termination.
- Contains: Python modules implementing Isaac Lab manager-based terms.
- Key files: `observations.py`, `rewards.py`, `track_manager.py`.

**arcproLab/scripts/:**
- Purpose: Operational scripts for interacting with the simulation.
- Contains: CLI scripts for training and verifying policies.
- Key files: `train_policy.py`, `verify_policy.py`, `verify_metric.py`.

**openStreetUSD/:**
- Purpose: Holds the 3D environments (Universal Scene Description).
- Contains: USD stages representing tracks and surrounding infrastructure.
- Key files: `no_graph_sim_cleaned.usd`.

**trash/tools/:**
- Purpose: A collection of standalone utility scripts for USD manipulation.
- Contains: Tools for physics repair, scaling, and scene cleaning.
- Key files: `clean_physics.py`, `rescale_usd.py`, `deep_bake.py`.

## Key File Locations

**Entry Points:**
- `arcproLab/scripts/train_policy.py`: Starts RL training.
- `arcproLab/scripts/verify_policy.py`: Runs a trained policy for visual check.
- `arcproLab/scripts/verify_metric.py`: Benchmarks policy performance.

**Configuration:**
- `arcproLab/arcpro_env_cfg.py`: Environment and physics settings.
- `arcproLab/arcpro_robot_cfg.py`: Robot articulation and actuator settings.

**Core Logic:**
- `arcproLab/mdp/track_manager.py`: Core logic for track-relative navigation.
- `arcproLab/mdp/observations.py`: Telemetry vector construction.

**Testing:**
- `tests/test_track_manager.py`: Validation of track waypoint logic.

## Naming Conventions

**Files:**
- `*_cfg.py`: Configuration files (e.g., `arcpro_env_cfg.py`).
- `verify_*.py`: Scripts used for performance measurement and visual checks.
- `test_*.py`: Pytest-compatible unit tests.

**Directories:**
- `mdp/`: Standard Isaac Lab naming for Markov Decision Process logic.
- `assets/`: 3D model storage.

## Where to Add New Code

**New Feature (e.g., new reward):**
- Implementation: `arcproLab/mdp/rewards.py`.
- Integration: Update `RewardCfg` in `arcproLab/arcpro_env_cfg.py`.

**New Environment/Track:**
- 3D Model: Place USD in `openStreetUSD/`.
- Configuration: Reference in `arcproLab/arcpro_env_cfg.py` within `ARCProSceneCfg.track`.

**New Component/Module:**
- Shared utility: `arcproLab/mdp/` if related to the RL loop.
- Standalone tool: `trash/tools/` (or create a new `tools/` directory if official).

**Utilities:**
- Shared helpers: `arcproLab/mdp/` for logic, `arcproLab/` for generic python utilities.

## Special Directories

**.planning/:**
- Purpose: Contains GSD project metadata, roadmaps, and codebase analysis.
- Generated: No (Created by Agent).
- Committed: Yes.

**trash/:**
- Purpose: Staging area for experimental and legacy scripts.
- Generated: No.
- Committed: Yes.

---

*Structure analysis: 2024-10-31*
