# Codebase Structure

**Analysis Date:** 2025-05-15

## Directory Layout

```
[project-root]/
├── arcproLab/          # Core RL Logic & Environments
│   ├── assets/         # Robot USD models (F1Tenth_Metric.usd)
│   ├── mdp/            # Markov Decision Process logic (obs, rewards, etc.)
│   ├── models/         # Pre-trained policy checkpoints (.pth, .zip)
│   ├── policy_stack/   # Policy architecture definitions
│   └── scripts/        # Training, verification, and utility scripts
├── openStreetUSD/      # Environment assets (1x metric scaled USDA/USD)
├── logs/               # TensorBoard logs and policy checkpoints
├── tests/              # Unit tests for core logic
└── .planning/          # GSD planning and research documentation
```

## Directory Purposes

**arcproLab/mdp/:**
- Purpose: Contains all logic linking physics to RL training.
- Contains: `observations.py`, `rewards.py`, `terminations.py`, `events.py`, `track_manager.py`.
- Key files: `track_centerline_1x.npy` (1x metric waypoints).

**arcproLab/scripts/:**
- Purpose: Operational scripts for interacting with simulation.
- Contains: `train_policy.py`, `verify_live.py`, `verify_metric.py`, `verify_spawn.py`.
- Key files: `verify_metric.py` (verifies 1.0x scale transition and joint velocities).

**openStreetUSD/:**
- Purpose: Storage for environmental assets.
- Contains: `no_graph_sim_clean_1x.usda` (Primary training environment).

**arcproLab/policy_stack/policies/:**
- Purpose: Neural network architecture implementations.
- Contains: `hierarchical_policy.py`, `fusion_policy.py`.

## Key File Locations

**Entry Points:**
- `train.sh`: Launch training via `arcproLab/scripts/train_policy.py`.
- `verify_sim.sh`: Launch headless verification via `arcproLab/scripts/verify_policy.py`.
- `run_gui_verify.sh`: Launch GUI-based verification via `arcproLab/scripts/verify_live.py`.

**Configuration:**
- `arcproLab/arcpro_env_cfg.py`: Main environment configuration (Scale, Sensors, MDP).
- `arcproLab/arcpro_robot_cfg.py`: Robot configuration (Metric USD path, mass: 20kg).

**Core Logic:**
- `arcproLab/mdp/observations.py`: Implements the 12-element telemetry protocol at 1x scale.
- `arcproLab/mdp/track_manager.py`: Manages 1x track waypoints and computes errors in meters.

**Testing:**
- `tests/test_track_manager.py`: Unit tests for waypoint logic.

## Naming Conventions

**Files:**
- [snake_case.py]: Logic and scripts.
- [snake_case_1x.usda]: 1x metric scaled asset definitions.

**Directories:**
- [snake_case/]: Standard directory naming.

## Where to Add New Code

**New Feature (RL Logic):**
- Primary code: `arcproLab/mdp/` (add to `observations.py`, `rewards.py`, or `terminations.py`).
- Config: `arcproLab/arcpro_env_cfg.py` (register the new term).

**New Component/Module:**
- Implementation: `arcproLab/` (create new subdirectory).

**Utilities:**
- Shared helpers: `arcproLab/scripts/` or `arcproLab/mdp/`.

## Special Directories

**.planning/:**
- Purpose: Project roadmap and architectural research.
- Committed: Yes.

**logs/:**
- Purpose: Output from training runs (TensorBoard, checkpoints).
- Committed: No.

---

*Structure analysis: 2025-05-15*
