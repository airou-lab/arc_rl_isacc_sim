# Codebase Structure

**Analysis Date:** 2026-03-26

## Directory Layout

```
[project-root]/
├── .github/          # GitHub Actions CI/CD workflows
├── .planning/        # Project planning, roadmap, and codebase documentation
├── arcproLab/        # Main application source code for Isaac Lab RL environment
│   ├── assets/       # Static assets, e.g., robot models
│   ├── mdp/          # Markov Decision Process (MDP) components for RL environment
│   ├── models/       # Trained RL policies or models
│   └── scripts/      # Executable scripts for training, verification, and utility
├── docs/             # High-level project documentation and research
├── openStreetUSD/    # USD assets for simulation environments (e.g., tracks)
├── tests/            # Unit and integration tests
├── trash/            # Contains various utility tools, likely for USD manipulation or debugging
└── venv/             # Python virtual environment
```

## Directory Purposes

**`.github/`:**
- Purpose: Contains GitHub Actions workflows for continuous integration and continuous deployment.
- Contains: YAML files defining CI/CD pipelines.
- Key files: `.github/workflows/ci.yml`

**`.planning/`:**
- Purpose: Stores project planning documents, including high-level requirements, roadmap, and automatically generated codebase analysis.
- Contains: Markdown documents for various planning aspects.
- Key files: `.planning/PROJECT.md`, `.planning/codebase/ARCHITECTURE.md`

**`arcproLab/`:**
- Purpose: The core Python package for defining and interacting with the Isaac Lab reinforcement learning environment.
- Contains: Python modules, configuration files, assets, and scripts related to the RL setup.
- Key files: `arcproLab/__init__.py`, `arcproLab/arcpro_env_cfg.py`, `arcproLab/arcpro_robot_cfg.py`

**`arcproLab/assets/`:**
- Purpose: Stores static assets used within the Isaac Lab simulation, such as robot descriptions.
- Contains: USD files or other model definitions.
- Key files: `arcproLab/assets/robot/`

**`arcproLab/mdp/`:**
- Purpose: Defines the individual components of the Reinforcement Learning Markov Decision Process.
- Contains: Python modules for observations, rewards, terminations, events, and policy wrappers.
- Key files: `arcproLab/mdp/observations.py`, `arcproLab/mdp/rewards.py`, `arcproLab/mdp/terminations.py`

**`arcproLab/models/`:**
- Purpose: Stores pre-trained machine learning models or RL policies.
- Contains: Model checkpoint files (e.g., `.pth`).
- Key files: `arcproLab/models/road_following_model.pth`

**`arcproLab/scripts/`:**
- Purpose: Contains executable Python scripts for running various operations, including training, verification, and utility functions related to the RL environment.
- Contains: Python scripts that serve as main entry points.
- Key files: `arcproLab/scripts/train_policy.py`, `arcproLab/scripts/verify_policy.py`

**`docs/`:**
- Purpose: Stores general project documentation, overviews, and research notes.
- Contains: Markdown documents.
- Key files: `docs/RL_OVERVIEW.md`, `docs/PROJECT.md`

**`openStreetUSD/`:**
- Purpose: Contains Universal Scene Description (USD) files that define the simulation environments, such as tracks and scenes.
- Contains: USD asset files.
- Key files: `openStreetUSD/arcpro_RL_open_street_sim.usd`

**`tests/`:**
- Purpose: Contains unit and integration tests for the codebase.
- Contains: Python test files.
- Key files: `tests/test_track_manager.py`

**`trash/`:**
- Purpose: This directory appears to contain a large collection of utility scripts, possibly for debugging, manipulating USD assets, or internal development tools. While named "trash", its contents suggest it's actively used for specialized tasks.
- Contains: Numerous Python scripts.
- Key files: `trash/tools/bake_track.py`, `trash/tools/measure_robot.py`

## Key File Locations

**Entry Points:**
- `train.sh`: Main shell script for initiating RL training.
- `verify_sim.sh`: Shell script for initiating simulation verification.
- `arcproLab/scripts/train_policy.py`: Python script for RL training.
- `arcproLab/scripts/verify_policy.py`: Python script for verifying trained policies.

**Configuration:**
- `requirements.txt`: Python package dependencies.
- `arcproLab/arcpro_env_cfg.py`: Environment configuration for Isaac Lab.
- `arcproLab/arcpro_robot_cfg.py`: Robot configuration for Isaac Lab.

**Core Logic:**
- `arcproLab/mdp/`: Directory containing core RL MDP logic.
- `arcproLab/mdp/track_manager.py`: Manages track data and generation.

**Testing:**
- `tests/test_track_manager.py`: Example unit test.

## Naming Conventions

**Files:**
- **Python modules:** `snake_case.py` (e.g., `rewards.py`, `train_policy.py`)
- **Configuration files:** `snake_case_cfg.py` (e.g., `arcpro_env_cfg.py`)
- **USD files:** `snake_case.usd` (e.g., `arcpro_RL_open_street_sim.usd`)
- **Shell scripts:** `snake_case.sh` (e.g., `train.sh`)

**Directories:**
- **Python packages:** `snake_case` (e.g., `arcproLab`, `mdp`, `scripts`)
- **Special purpose:** `.dot_prefixed` for hidden/config directories (e.g., `.github`, `.planning`)

## Where to Add New Code

**New Feature (RL environment component):**
- Primary code (e.g., new observation type): `arcproLab/mdp/`
- Configuration for new component: Update relevant `arcproLab/*_env_cfg.py` or `arcproLab/*_robot_cfg.py`

**New Training/Verification Script:**
- Implementation: `arcproLab/scripts/`
- Associated shell wrapper (if needed): Project root (e.g., `new_script.sh`)

**New Robot Model:**
- USD model definition: `arcproLab/assets/robot/`
- Configuration: Create or update a `arcproLab/*_robot_cfg.py` file.

**New Track/Simulation Environment:**
- USD scene definition: `openStreetUSD/`
- Track generation logic (if procedural): `arcproLab/generate_track.py` or a new utility in `arcproLab/`

**Utilities:**
- General helper functions: `arcproLab/utils/` (if such a directory were to be created, otherwise perhaps `arcproLab/` directly or `trash/tools/` for specialized tools).

## Special Directories

**`.venv/` or `venv/`:**
- Purpose: Python virtual environment for managing project dependencies.
- Generated: Yes, by `python -m venv` or similar.
- Committed: No (typically listed in `.gitignore`).

**`__pycache__/`:**
- Purpose: Stores bytecode compiled Python files (`.pyc`).
- Generated: Yes, automatically by Python.
- Committed: No (typically listed in `.gitignore`).

**`.pytest_cache/`:**
- Purpose: Cache directory for `pytest` test runner.
- Generated: Yes, by `pytest`.
- Committed: No (typically listed in `.gitignore`).

---

*Structure analysis: 2026-03-26*
