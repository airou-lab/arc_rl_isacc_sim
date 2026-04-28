# Codebase Structure

**Analysis Date:** 2024-04-23

## Directory Layout

```
arcpro_rl_isacc_sim/
├── arcproLab/              # Main Isaac Lab environment package
│   ├── assets/             # Robot USD models and textures
│   ├── mdp/                # Markov Decision Process components
│   │   ├── actions.py      # Action mappings
│   │   ├── observations.py # Telemetry vector construction
│   │   ├── rewards.py      # Reward functions
│   │   ├── terminations.py # Episode end conditions
│   │   ├── track_manager.py# Boundary and waypoint logic
│   │   └── road_graph.py   # High-level navigation intent
│   ├── scripts/            # Training and verification scripts
│   ├── arcpro_env_cfg.py   # Core Isaac Lab environment config
│   └── arcpro_robot_cfg.py # Robot actuator and physics config
├── openStreetUSD/          # Track and world assets
├── policy_stack/           # Neural network and policy implementations
├── logs/                   # Training logs and checkpoints
├── docs/                   # Project documentation
└── .planning/              # Project planning and codebase maps
```

## Directory Purposes

**arcproLab/:**
- Purpose: The core logic of the RL environment tailored for Isaac Lab.
- Contains: Environment configurations, robot definitions, and MDP managers.
- Key files: `arcpro_env_cfg.py`, `arcpro_robot_cfg.py`.

**arcproLab/mdp/:**
- Purpose: "Brain" of the environment dynamics and observation space.
- Contains: Logic for calculating rewards, errors, and intent.
- Key files: `track_manager.py`, `observations.py`.

**arcproLab/scripts/:**
- Purpose: Operational scripts for the project.
- Contains: Training (`train_policy.py`), verification (`verify_policy.py`), and utility scripts (`audit_assets.py`).

**openStreetUSD/:**
- Purpose: 3D scene storage.
- Contains: USD/USDA files representing the track at 1.0x metric scale.
- Key files: `no_graph_sim_clean_1x.usda`.

**policy_stack/:**
- Purpose: High-level policy architecture.
- Contains: Hierarchical PPO implementations and feature extractors.

## Key File Locations

**Entry Points:**
- `arcproLab/scripts/train_policy.py`: Starts RL training.
- `arcproLab/scripts/verify_policy.py`: Runs inference for verification.
- `arcproLab/scripts/verify_metric.py`: Verifies scaling and units.

**Configuration:**
- `arcproLab/arcpro_env_cfg.py`: Environment parameters (camera, scene, rewards).
- `arcproLab/arcpro_robot_cfg.py`: Robot physical parameters.

**Core Logic:**
- `arcproLab/mdp/track_manager.py`: Path and boundary calculations.
- `arcproLab/mdp/road_graph.py`: Navigation intent logic.

**Testing:**
- `arcproLab/scripts/verify_*.py`: Suite of functional verification scripts.
- `tests/`: Pytest-based unit tests for core utilities (e.g., `test_track_manager.py`).

## Naming Conventions

**Files:**
- Snake Case: `arcpro_env_cfg.py`, `track_manager.py`.

**Directories:**
- Snake Case or Camel Case: `arcproLab`, `policy_stack`.

## Where to Add New Code

**New Feature (e.g., Traffic Lights):**
- Primary code: `arcproLab/mdp/events.py` or new file in `mdp/`.
- Config: `arcproLab/arcpro_env_cfg.py` (add to Scene or Managers).

**New Component/Module:**
- Implementation: `arcproLab/mdp/` (for MDP logic) or `policy_stack/` (for NNs).

**Utilities:**
- Shared helpers: `arcproLab/mdp/` or `arcproLab/scripts/` depending on scope.

## Special Directories

**logs/:**
- Purpose: Contains ephemeral data (checkpoints, tensorboard).
- Generated: Yes.
- Committed: No (usually ignored by `.gitignore`).

---

*Structure analysis: 2024-04-23*
