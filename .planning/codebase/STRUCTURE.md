# Codebase Structure

**Analysis Date:** 2025-05-20

## Directory Layout

```
[project-root]/
├── arcproLab/          # Core RL Logic & Environments
│   ├── assets/         # Robot USD models (F1Tenth_Metric.usd)
│   ├── mdp/            # Markov Decision Process logic (obs, rewards, terminations, events)
│   ├── models/         # Pre-trained policy checkpoints (.pth, .zip)
│   ├── policy_stack/   # Policy architecture definitions (fusion, hierarchical)
│   └── scripts/        # Operational scripts (train, verify, audit, cleanup)
├── openStreetUSD/      # Environment assets (1x metric scaled USDA/USD)
├── docs/               # Project documentation (Legacy and GSD)
├── logs/               # TensorBoard logs and policy checkpoints
├── tests/              # Unit tests (TrackManager)
├── trash/              # [REDUNDANT] Legacy scripts and assets
└── .planning/          # GSD planning, roadmap, and codebase maps
```

## Directory Purposes

**arcproLab/mdp/:**
- Purpose: Bridge between physics simulation and RL algorithms.
- Contains: `observations.py` (telemetry), `rewards.py`, `terminations.py`, `events.py` (reset logic), `track_manager.py` (lane logic).
- Key files: `track_manager.py` (dynamic centerline generation).

**arcproLab/scripts/:**
- Purpose: Tools for development, training, and verification.
- Contains: `train_policy.py`, `verify_live.py`, `audit_live.py`, `clean_terrain_v2.py`.
- Key files: `train_policy.py` (SB3 PPO entry point).

**openStreetUSD/:**
- Purpose: Primary source for environment geometry.
- Contains: `no_graph_sim_clean_1x.usda` (1.0x metric training track).

**arcproLab/assets/robot/:**
- Purpose: Robot URDF/USD definitions.
- Contains: `F1Tenth_Metric.usd` (20kg metric-scale robot).

## Key File Locations

**Entry Points:**
- `train.sh`: Wrapper for `arcproLab/scripts/train_policy.py`.
- `verify_sim.sh`: Wrapper for `arcproLab/scripts/verify_policy.py` (Headless).
- `run_gui_verify.sh`: Wrapper for `arcproLab/scripts/verify_live.py` (GUI).

**Configuration:**
- `arcproLab/arcpro_env_cfg.py`: Environment parameters (Observation/Reward/Termination config).
- `arcproLab/arcpro_robot_cfg.py`: Robot physical parameters (Actuators, Sensors, Drivetrain).

**Core Logic:**
- `arcproLab/mdp/track_manager.py`: Singleton managing waypoints and errors.
- `arcproLab/mdp/observations.py`: 12-element telemetry protocol implementation.

**Testing:**
- `tests/test_track_manager.py`: Validation of waypoint math.

## Naming Conventions

**Files:**
- [snake_case.py]: Modules and scripts.
- [snake_case_1x.usda]: Assets scaled for metric physics.

**Directories:**
- [snake_case/]: Standard organization.

## Where to Add New Code

**New Feature (RL Logic):**
- Implementation: `arcproLab/mdp/`
- Configuration: Add term to `ARCProEnvCfg` in `arcproLab/arcpro_env_cfg.py`.

**New Utility Script:**
- Implementation: `arcproLab/scripts/`. Follow `verify_*` or `audit_*` naming.

**New Robot Asset:**
- Implementation: `arcproLab/assets/robot/`.

## Special Directories

**trash/:**
- Purpose: Staging area for files pending deletion.
- Note: Should be emptied periodically.

**logs/:**
- Purpose: SB3 training output. Ignored by git.

---

*Structure analysis: 2025-05-20*
