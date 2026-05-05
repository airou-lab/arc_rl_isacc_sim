# Coding Conventions

**Analysis Date:** 2025-01-23

## Naming Patterns

**Files:**
- snake_case for all source files (e.g., `observations.py`, `track_manager.py`).
- Test files prefixed with `test_` (e.g., `test_track_manager.py`).

**Functions:**
- snake_case for all functions (e.g., `get_telemetry_vector()`, `compute_marker_distances()`).

**Variables:**
- snake_case for local variables and parameters (e.g., `asset_cfg`, `env_origins`).

**Types:**
- PascalCase for classes (e.g., `TrackManager`, `AgentNode`, `WorkerNode`).
- UPPER_SNAKE_CASE for constants and index definitions (e.g., `TELEMETRY_INDICES`, `IDX_LAT_ERR`).

## Code Style

**Formatting:**
- Standard Python (PEP 8) style is followed.
- Indentation: 4 spaces.

**Linting:**
- Not explicitly configured in the root, but `arcproLab/policy_stack/pytest.ini` disables several ament/colcon linting plugins, suggesting a ROS2 environment might be involved but suppressed for core tests.

## Import Organization

**Order:**
1. Standard library imports (e.g., `import os`, `import sys`).
2. Major external dependencies (e.g., `import torch`, `import numpy as np`).
3. Isaac Lab / Isaac Sim specific imports (e.g., `from isaaclab.managers import SceneEntityCfg`).
4. Internal project imports (e.g., `from mdp.road_graph import get_road_graph`).

**Path Aliases:**
- `sys.path.insert(0, ...)` is frequently used in scripts and tests to ensure `arcproLab` or local directories are discoverable.

## Error Handling

**Patterns:**
- `torch.isnan(reward)` checks in reward functions to ensure stability.
- `try-except` blocks for optional imports (e.g., `HAS_TORCH` checks in `test_all_so_far.py`).
- Existence checks for files and directories using `os.path.exists()` before loading data.

## Logging

**Framework:** `print()`

**Patterns:**
- Prefixed print statements for component-level logging: `[TrackManager]`, `[DIAGNOSTIC]`.
- Conditional logging based on debug flags or random sampling (e.g., `if torch.rand(1).item() < 0.05:`).

## Comments

**When to Comment:**
- Complexity: Mathematical formulas (e.g., yaw from quaternion) are commented with the formula.
- Protocols: Index mappings for telemetry are documented in docstrings.

**JSDoc/TSDoc:**
- Triple-quoted docstrings are used for functions and classes, often providing usage examples or parameter descriptions.

## Function Design

**Size:**
- Generally small and focused on a single task (e.g., one function per reward or termination criteria).

**Parameters:**
- Heavily use type hinting (e.g., `env: ManagerBasedRLEnv`, `asset_cfg: SceneEntityCfg`).
- Default parameters are common for configuration objects.

**Return Values:**
- Most RL-related functions (rewards, observations, terminations) return `torch.Tensor` for GPU acceleration.

## Module Design

**Exports:**
- Standard Python module exports.

**Barrel Files:**
- `__init__.py` files are used to organize subpackages like `mdp/`.

---

*Convention analysis: 2025-01-23*
