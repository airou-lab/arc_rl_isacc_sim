# Codebase Structure

**Analysis Date:** 2025-04-18

## Directory Layout

```
[project-root]/
├── arcproLab/          # Core RL logic and environment
│   ├── assets/         # Simulation assets (F1Tenth_Metric.usd)
│   ├── mdp/            # RL interface logic (Obs, Rew, Done)
│   │   ├── events.py   # Spawning and domain randomization
│   │   ├── observations.py # 12-float telemetry and vision input
│   │   ├── rewards.py  # Gaussian-weighted reward functions
│   │   ├── terminations.py # Episode termination conditions
│   │   └── track_manager.py # Absolute waypoint management
│   ├── models/         # Trained model checkpoints (.pth)
│   ├── policy_stack/   # Policy architectures (Hierarchical, Fusion)
│   ├── scripts/        # Training, verification, and audit tools
│   ├── arcpro_env_cfg.py # Main environment configuration
│   └── arcpro_robot_cfg.py # Main robot articulation configuration
├── openStreetUSD/      # Map assets (no_graph_sim.usd)
├── docs/               # Documentation and roadmaps
├── tests/              # Pytest logic tests (test_track_manager.py)
├── trash/              # Legacy tools and experimental scripts
├── .github/            # GitHub CI/CD workflows
├── requirements.txt    # Dependencies
├── train.sh            # Training launcher
└── verify_sim.sh       # Headless simulation verification
```

## Directory Purposes

**arcproLab/:**
- Purpose: Primary workspace for RL development in IsaacLab.
- Contains: Configuration classes, MDP logic, and entry point scripts.

**arcproLab/mdp/:**
- Purpose: Bridge between Isaac Sim physics and the RL policy.
- Contains: Logic for translating raw sensor data into the **12-float telemetry vector**.
- Key files: `observations.py`, `track_manager.py`, `rewards.py`.

**arcproLab/assets/:**
- Purpose: Local USD assets used in simulations.
- Contains: `robot/F1Tenth_Metric.usd`.

**openStreetUSD/:**
- Purpose: Storage for world geometry and road networks.
- Contains: `no_graph_sim.usd` (The primary track used in `arcpro_env_cfg.py`).

**arcproLab/scripts/:**
- Purpose: Standalone execution scripts for common tasks.
- Key files: `train_policy.py`, `verify_policy.py`, `audit_assets.py`.

## Key File Locations

**Entry Points:**
- `arcproLab/scripts/train_policy.py`: Main SB3 training script.
- `arcproLab/scripts/verify_policy.py`: Model evaluation script.

**Configuration:**
- `arcproLab/arcpro_env_cfg.py`: Environment and Scene setup (defines 8x robot scale).
- `arcproLab/arcpro_robot_cfg.py`: Robot physical parameters (20kg mass, 4WD).

**Core Logic:**
- `arcproLab/mdp/track_manager.py`: Critical waypoint logic and USD mesh sampling.

**Testing:**
- `tests/test_track_manager.py`: Unit tests for waypoint logic.

## Naming Conventions

**Files:**
- snake_case: `arcpro_env_cfg.py`.

**Directories:**
- camelCase/snake_case: `arcproLab/`, `openStreetUSD/`.

## Where to Add New Code

**New Feature (MDP):**
- Primary code: `arcproLab/mdp/`.
- Tests: `tests/`.

**New Component/Module:**
- Implementation: `arcproLab/` or `arcproLab/policy_stack/`.

**Utilities:**
- Simulation-related: `arcproLab/mdp/`.
- Deployment-related: `arcproLab/scripts/`.

## Special Directories

**trash/tools/:**
- Purpose: Legacy or one-off maintenance scripts.
- Committed: Yes.

**openStreetUSD/archive/:**
- Purpose: Historical or experimental track assets.
- Committed: Yes.

---

*Structure analysis: 2025-04-18*
