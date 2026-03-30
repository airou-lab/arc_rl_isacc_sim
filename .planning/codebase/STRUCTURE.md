# Codebase Structure

**Analysis Date:** 2024-10-24

## Directory Layout

```
arc_rl_isaac_sim/
├── arcproLab/              # Main application package
│   ├── assets/             # Robot USD assets (F1Tenth_Metric.usd)
│   ├── mdp/                # Markov Decision Process logic (obs, rewards, etc.)
│   │   ├── track_manager.py # Coordinates and track-specific logic
│   │   └── track_centerline.npy # Waypoint data for errors
│   ├── scripts/            # Executable scripts for training and verification
│   ├── arcpro_env_cfg.py   # Main Isaac Lab environment config
│   ├── arcpro_robot_cfg.py # F1Tenth robot configuration
│   └── verify.py           # GUI verification script
├── docs/                   # Project documentation and phase history
├── openStreetUSD/          # USD files for the simulation track
├── tests/                  # Unit tests (track_manager, etc.)
├── trash/                  # Deprecated or experimental files (pre-metric scaling)
└── .planning/              # GSD internal planning documents
```

## Directory Purposes

**arcproLab/:**
- Purpose: Root package for the Isaac Lab implementation.
- Contains: Configuration, assets, and MDP components.
- Key files: `arcpro_env_cfg.py`, `arcpro_robot_cfg.py`.

**arcproLab/mdp/:**
- Purpose: Implements the reinforcement learning interface.
- Contains: Python modules for observations, rewards, events, and terminations.
- Key files: `observations.py`, `track_manager.py`.

**arcproLab/scripts/:**
- Purpose: Executable scripts for interacting with the simulation.
- Contains: Training, verification, and metric-checking scripts.
- Key files: `train_policy.py`, `verify_policy.py`.

**openStreetUSD/:**
- Purpose: Stores track environment USD files.
- Contains: Cleaned and final versions of track meshes.
- Key files: `no_graph_sim_cleaned.usd`.

## Key File Locations

**Entry Points:**
- `arcproLab/scripts/train_policy.py`: Training entry point.
- `arcproLab/verify.py`: GUI verification of robot and environment.

**Configuration:**
- `arcproLab/arcpro_env_cfg.py`: Environment parameters (gravity, dt, PhysX).
- `arcproLab/arcpro_robot_cfg.py`: Robot asset path, scales, and actuators.

**Core Logic:**
- `arcproLab/mdp/track_manager.py`: Waypoint management and track error calculation.
- `arcproLab/mdp/observations.py`: 12-element telemetry vector construction.

**Testing:**
- `tests/test_track_manager.py`: Logic verification for track geometry calculations.

## Naming Conventions

**Files:**
- Configuration: `*_cfg.py`
- Scripts: Verb-noun e.g., `train_policy.py`, `verify_spawn.py`.

**Directories:**
- Python packages: `camelCase` (e.g., `arcproLab`) or standard lowercase.
- Data storage: `snake_case` or `camelCase`.

## Where to Add New Code

**New Feature (MDP):**
- Implementation: `arcproLab/mdp/` (create new file or add to existing if applicable).
- Configuration: Update `arcproLab/arcpro_env_cfg.py`.

**New Observation Term:**
- Implementation: `arcproLab/mdp/observations.py`.
- Config: Register in `ObservationCfg` within `arcproLab/arcpro_env_cfg.py`.

**New Robot Type:**
- Implementation: `arcproLab/arcpro_robot_cfg.py` (add new `ArticulationCfg`).
- Asset: `arcproLab/assets/robot/`.

**Utilities:**
- Implementation: `arcproLab/mdp/` or new top-level directory if broadly useful.

## Special Directories

**trash/:**
- Purpose: Contains deprecated files from previous phases (e.g., pre-metric scaling experiments).
- Generated: No.
- Committed: Yes.

**docs/:**
- Purpose: Documentation and verification history.
- Generated: No.
- Committed: Yes.

---

*Structure analysis: 2024-10-24*
