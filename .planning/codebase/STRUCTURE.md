# Codebase Structure

**Analysis Date:** 2024-04-12

## Directory Layout

```
[project-root]/
├── arcproLab/          # Core RL Logic & Environments
│   ├── assets/         # Robot USD models (e.g., F1Tenth_Metric.usd)
│   ├── mdp/            # Markov Decision Process logic (obs, rewards, etc.)
│   ├── models/         # Pre-trained policy checkpoints (.pth)
│   ├── policy_stack/   # Policy architecture definitions (Hierarchical, Fusion)
│   └── scripts/        # Training, verification, and utility scripts
├── openStreetUSD/      # Environment assets (Road tracks, USDAs)
├── logs/               # TensorBoard logs and policy checkpoints
├── tests/              # Unit tests for core components
└── .planning/          # GSD planning and research documentation
```

## Directory Purposes

**arcproLab/mdp/:**
- Purpose: Contains all the logic that links physics to RL training.
- Contains: `observations.py`, `rewards.py`, `terminations.py`, `events.py`.
- Key files: `track_manager.py` (waypoint navigation), `visual_analytics.py` (telemetry GUI).

**arcproLab/scripts/:**
- Purpose: Operational scripts for interacting with the simulation.
- Contains: `train_policy.py`, `verify_policy.py`, `apply_usd_scale.py`.
- Key files: `verify_metric.py` (verifies 1.0x scale transition).

**openStreetUSD/:**
- Purpose: Storage for environmental assets used in simulation.
- Contains: `no_graph_sim_1x.usda`, `no_graph_sim_clean_1x.usda`.
- Key files: `no_graph_sim_clean_1x.usda` (The 'Void' environment track).

**arcproLab/policy_stack/policies/:**
- Purpose: Neural network architecture implementations.
- Contains: `hierarchical_policy.py`, `fusion_policy.py`.

## Key File Locations

**Entry Points:**
- `train.sh`: Shell script to launch training via `arcproLab/scripts/train_policy.py`.
- `verify_sim.sh`: Shell script to launch verification via `arcproLab/scripts/verify_policy.py`.

**Configuration:**
- `arcproLab/arcpro_env_cfg.py`: Main environment configuration (Scale, Sensors, MDP).
- `arcproLab/arcpro_robot_cfg.py`: Robot configuration (USD path, physics parameters).

**Core Logic:**
- `arcproLab/mdp/observations.py`: Implements the 12-element telemetry protocol.
- `arcproLab/mdp/track_manager.py`: Manages track waypoints and computes errors in meters.

**Testing:**
- `tests/test_track_manager.py`: Unit tests for waypoint logic.

## Naming Conventions

**Files:**
- [snake_case.py]: Logic and scripts (e.g., `track_manager.py`)
- [snake_case.usda]: Asset definitions (e.g., `no_graph_sim_clean_1x.usda`)

**Directories:**
- [snake_case/]: Standard directory naming (e.g., `policy_stack/`)

## Where to Add New Code

**New Feature (RL Logic):**
- Primary code: `arcproLab/mdp/` (add to `observations.py`, `rewards.py`, or `terminations.py`)
- Config: `arcproLab/arcpro_env_cfg.py` (register the new term)

**New Component/Module:**
- Implementation: `arcproLab/` (create a new subdirectory if necessary)

**Utilities:**
- Shared helpers: `arcproLab/scripts/` or `arcproLab/mdp/` depending on scope.

## Special Directories

**.planning/:**
- Purpose: Project roadmap, phase tracking, and architectural research.
- Generated: No
- Committed: Yes

**logs/:**
- Purpose: Output from training runs.
- Generated: Yes
- Committed: No (usually gitignored)

---

*Structure analysis: 2024-04-12*
