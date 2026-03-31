# Codebase Structure

**Analysis Date:** 2024-10-24

## Directory Layout

```
arcpro_rl_isacc_sim/
├── arcproLab/          # Core environment package
│   ├── assets/         # Robot USD models
│   ├── mdp/            # RL logic (Observation, Reward, Termination)
│   ├── models/         # Trained policy neural networks
│   └── scripts/        # Training and verification executables
├── openStreetUSD/      # Environment/Track USD assets
├── tests/              # Unit tests for MDP logic
├── trash/              # Archive of deprecated scripts and legacy tools
│   └── tools/          # Extensive library of USD/Physics utility scripts
└── docs/               # Documentation for the RL system
```

## Directory Purposes

**arcproLab/:**
- Purpose: Main Python package containing the Isaac Lab environment implementation.
- Contains: Configuration classes and subpackages for robot assets and RL logic.
- Key files: `arcpro_env_cfg.py`, `arcpro_robot_cfg.py`.

**arcproLab/mdp/:**
- Purpose: Markov Decision Process (MDP) components for Reinforcement Learning.
- Contains: Functions for rewards, observations, and termination conditions.
- Key files: `track_manager.py` (Core geometry handler), `observations.py`.

**arcproLab/scripts/:**
- Purpose: Entry point scripts for running the simulation.
- Contains: Python scripts for training policies and verifying them in Isaac Sim.
- Key files: `train_policy.py`, `verify_policy.py`.

**openStreetUSD/:**
- Purpose: Stores the 3D environment assets imported from OpenStreetMap or generated via USD pipelines.
- Contains: `.usd` files representing the track geometry and physics.
- Key files: `no_graph_sim_final.usd`.

**trash/tools/:**
- Purpose: A repository of specialized utility scripts for USD manipulation, physics repair, and asset auditing.
- Contains: Python scripts for scaling USDs, fixing joints, auditing mass, and cleaning physics.
- Key files: `nuclear_physics_fix.py`, `repair_materials_gsd.py`, `scale_usd.py`.

## Key File Locations

**Entry Points:**
- `arcproLab/scripts/train_policy.py`: Starts the RL training process.
- `arcproLab/scripts/verify_policy.py`: Runs a trained policy for evaluation.

**Configuration:**
- `arcproLab/arcpro_env_cfg.py`: Defines the Isaac Lab environment setup (Scene, Managers, Sim Settings).
- `arcproLab/arcpro_robot_cfg.py`: Defines the robot articulation, actuators, and initial state.

**Core Logic:**
- `arcproLab/mdp/track_manager.py`: Logic for sampling the USD track and providing centerline coordinates.
- `arcproLab/mdp/observations.py`: Logic for constructing the telemetry vector from simulation state.

**Testing:**
- `tests/test_track_manager.py`: Unit tests for the waypoint sampling and error calculation logic.

## Naming Conventions

**Files:**
- Snake Case: `train_policy.py`, `arcpro_env_cfg.py`.

**Directories:**
- Camel Case (Internal Packages): `arcproLab`.
- Snake Case (Resource Directories): `openStreetUSD`.

## Where to Add New Code

**New RL Feature (e.g., Collision Reward):**
- Primary code: `arcproLab/mdp/rewards.py` (add a new reward function).
- Configuration: `arcproLab/arcpro_env_cfg.py` (add the function to `RewardCfg`).

**New Sensor/Robot Change:**
- Implementation: `arcproLab/arcpro_robot_cfg.py` (modify `ArticularionCfg` or `ActuatorCfg`).
- Assets: Place new USD models in `arcproLab/assets/robot/`.

**USD Manipulation Utility:**
- Shared helpers: `trash/tools/` (or create a new `utils/` directory if it becomes a permanent part of the workflow).

## Special Directories

**openStreetUSD/archive/:**
- Purpose: Contains legacy or intermediate versions of the environment USD.
- Generated: No (manually curated)
- Committed: Yes

**arcproLab/models/:**
- Purpose: Stores trained PyTorch model weights (`.pth`).
- Generated: Yes (during training)
- Committed: Yes (for pre-trained policies)

---

*Structure analysis: 2024-10-24*
