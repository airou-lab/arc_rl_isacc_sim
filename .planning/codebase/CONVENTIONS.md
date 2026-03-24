# Coding Conventions

**Analysis Date:** 2024-05-24

## Naming Patterns

**Files:**
- `snake_case.py`: Used for all Python source files. Example: `arcpro_env_cfg.py`, `track_manager.py`.

**Functions:**
- `snake_case()`: Standard for all functions and methods. Example: `get_telemetry_vector()`, `load_waypoints()`.

**Variables:**
- `snake_case`: Standard for all variables and class members. Example: `lat_err`, `head_err`, `self.device`.

**Types/Classes:**
- `PascalCase`: Standard for class definitions. Example: `TrackManager`, `ARCProEnvCfg`.
- Type hints: Extensively used for function arguments and return values. Example: `(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor`.

## Code Style

**Formatting:**
- PEP8: Generally followed for indentation (4 spaces), spacing, and naming.
- `flake8`: Referenced in `requirements.txt` for linting.
- Line length: Some lines exceed the standard 79/88 characters, particularly in configuration and docstrings.

**Linting:**
- `flake8`: Recommended tool for linting as per `requirements.txt`.

## Import Organization

**Order:**
1. Standard library: `import os`, `import sys`.
2. Third-party libraries: `import torch`, `import numpy as np`.
3. Isaac Lab libraries: `from isaaclab.managers import ...`, `import isaaclab.sim as sim_utils`.
4. Project-specific modules: `from arcpro_robot_cfg import ARCPRO_ROBOT_CFG`, `import mdp.observations as mdp_obs`.

**Path Aliases:**
- None detected, but `sys.path.insert(0, ...)` is used in tests to resolve `arcproLab` paths.

## Error Handling

**Patterns:**
- `try-except` blocks: Used for defensive programming, particularly when interacting with the Isaac Sim/USD environment which might not be fully initialized or missing attributes.
- NaN checks: Common for reward and observation tensors to prevent training crashes. `torch.isnan()` is used to create masks and zero out NaNs.

## Logging

**Framework:** `print()` or Isaac Sim's internal logging.

**Patterns:**
- Informative status messages: Prefixed with module name, e.g., `[TrackManager] Loaded ... waypoints`.
- Warning/Error messages: Used when fallback logic is triggered, e.g., `[TrackManager] Warning: USD sampling failed`.

## Comments

**When to Comment:**
- Header: License/Copyright header (Isaac Lab Project Developers) at the top of each file.
- Docstrings: For every class and public function/method, describing purpose, arguments, and return values.
- Inline: For complex logic, particularly mathematical calculations (e.g., quaternion to yaw conversion).

**JSDoc/TSDoc:**
- Python standard docstrings (`"""Docstring content"""`) are used.

## Function Design

**Size:** Generally focused and modular, particularly in the `mdp` (Markov Decision Process) folder for rewards and observations.

**Parameters:** Use type hints and default values where appropriate. Often take `env: ManagerBasedRLEnv` as the first argument.

**Return Values:** Typically `torch.Tensor` for RL-related functions, matching the vectorized nature of the simulation.

## Module Design

**Exports:** Direct exports of functions and classes. 

**Barrel Files:** `__init__.py` files are present in subdirectories (e.g., `arcproLab/mdp/__init__.py`), though their usage varies.

---

*Convention analysis: 2024-05-24*
