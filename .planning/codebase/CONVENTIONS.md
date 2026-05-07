# Coding Conventions

**Analysis Date:** 2024-11-20

## Naming Patterns

**Files:**
- Snake_case for Python scripts and modules (e.g., `track_manager.py`).

**Functions:**
- Snake_case for function and method names (e.g., `ensure_synced()`, `load_cache()`).

**Variables:**
- Snake_case for local variables and attributes (e.g., `self.yellow_tensor`, `wp_np`).

**Types:**
- CamelCase for classes (e.g., `TrackManager`, `RoadGraph`).
- standard Python typing hints used (e.g., `device: str = "cuda:0"`).

## Code Style

**Formatting:**
- Follows standard Python (PEP 8) style.
- BSD-3-Clause license headers at the top of many files.

**Linting:**
- `pytest.ini` excludes several `ament_lint` and `flake8` plugins, suggesting a ROS-lite or customized environment.

## Import Organization

**Order:**
1. Standard library (e.g., `os`, `sys`).
2. Third-party libraries (e.g., `torch`, `numpy`).
3. Internal project modules (e.g., `from isaaclab.utils import ...`).

**Path Aliases:**
- `sys.path.append` is used in some scripts to ensure local package availability (e.g., in `arcpro_env_cfg.py`).

## Error Handling

**Patterns:**
- Try-except blocks for filesystem operations (`load_cache`).
- Print statements for warnings and initialization status (e.g., `[TrackManager] Building high-fidelity...`).

## Logging

**Framework:** `print()` for console output; Tensorboard for training metrics.

**Patterns:**
- Prefixed log messages: `[Component] Message`.

## Comments

**When to Comment:**
- Docstrings for classes describing their purpose.
- In-line comments for complex logic (e.g., yaw wrap-around handling in `track_manager.py`).

**JSDoc/TSDoc:**
- Not applicable (Python project). Using standard docstrings.

## Function Design

**Size:** Mixed; some complex initialization methods (like `ensure_synced`) are large but centralized.

**Parameters:** Use of default values and type hints is common.

**Return Values:** Explicit return types not always specified in method signatures but generally consistent.

## Module Design

**Exports:** Standard Python module exports.

**Barrel Files:** `__init__.py` used to mark directories as packages.

---

*Convention analysis: 2024-11-20*
