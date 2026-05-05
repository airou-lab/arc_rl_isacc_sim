# Codebase Structure

**Analysis Date:** 2024-05-18

## Directory Layout

```
arc_rl_isacc_sim/
├── arcproLab/              # Main package for Isaac Lab environment
│   ├── assets/             # Robot USD assets and models
│   ├── mdp/                # Markov Decision Process logic (rewards, observations, etc.)
│   ├── policy_stack/       # Policy architecture and agent training logic
│   ├── scripts/            # Training, verification, and utility scripts
│   ├── arcpro_env_cfg.py   # Main environment configuration
│   └── arcpro_robot_cfg.py # Robot articulation and actuator configuration
├── docs/                   # Project documentation and specifications
├── openStreetUSD/          # Simulation track USD files and artifacts
├── .planning/              # Project planning and codebase analysis
└── tests/                  # Unit tests for core components
```

## Directory Purposes

**arcproLab/:**
- Purpose: Root package for the RL environment implementation.
- Contains: Environment configurations and sub-packages for MDP and Policy.
- Key files: `arcpro_env_cfg.py`, `arcpro_robot_cfg.py`.

**arcproLab/mdp/:**
- Purpose: Contains modular components for the Reward, Observation, and Termination managers.
- Contains: logic for track management, road graphs, and spawner overrides.
- Key files: `track_manager.py`, `road_graph.py`, `rewards.py`, `terminations.py`.

**arcproLab/policy_stack/:**
- Purpose: Implementation of the RL agent's brain and training wrappers.
- Contains: Custom policies, feature extractors, and environment bridges.
- Key files: `policies/hierarchical_policy.py`, `wrappers/waypoint_tracking_wrapper.py`.

**arcproLab/scripts/:**
- Purpose: Operational scripts for the lifecycle of the RL agent.
- Contains: Training, manual drive verification, and USD asset audits.
- Key files: `train_policy.py`, `verify_policy.py`, `verify_drive.py`.

**openStreetUSD/:**
- Purpose: Storage for the 3D environments (tracks).
- Contains: `.usd` and `.usda` files used as the simulation stage.
- Key files: `no_graph_sim_clean_1x_flattened.usda`.

## Key File Locations

**Entry Points:**
- `arcproLab/scripts/train_policy.py`: Main entry point for training the hierarchical policy.
- `arcproLab/scripts/verify_policy.py`: Evaluates a trained checkpoint.
- `arcproLab/scripts/verify_drive.py`: Teleoperation script for manual environment testing.

**Configuration:**
- `arcproLab/arcpro_env_cfg.py`: Defines physics, sensors, and MDP manager bindings.
- `arcproLab/arcpro_robot_cfg.py`: Defines the F1Tenth robot's physical properties and actuators.

**Core Logic:**
- `arcproLab/mdp/track_manager.py`: Handles road-relative geometric calculations.
- `arcproLab/policy_stack/policies/hierarchical_policy.py`: The core HPPP neural network architecture.

**Testing:**
- `tests/test_track_manager.py`: Unit tests for the track manager logic.

## Naming Conventions

**Files:**
- Python files: Snake case (e.g., `track_manager.py`).
- USD files: Snake case, often with scale or status suffixes (e.g., `no_graph_sim_clean_1x.usda`).

**Directories:**
- Python packages: Snake case (e.g., `policy_stack`).

## Where to Add New Code

**New Feature (RL Logic):**
- Primary code: `arcproLab/mdp/` (add a new module for the feature and register it in `arcpro_env_cfg.py`).
- Tests: `tests/` (create a corresponding `test_*.py` file).

**New Component/Module (Policy):**
- Implementation: `arcproLab/policy_stack/policies/` or `arcproLab/policy_stack/wrappers/`.

**Utilities:**
- Shared helpers: `arcproLab/scripts/` if it's a tool, or a new module in `arcproLab/mdp/` if it's runtime-related.

## Special Directories

**logs/:**
- Purpose: Contains training logs, Tensorboard data, and checkpoints.
- Generated: Yes
- Committed: No (mostly)

**arcproLab/debug_frames/:**
- Purpose: Contains visual debug snapshots saved during training.
- Generated: Yes
- Committed: No

---

*Structure analysis: 2024-05-18*
