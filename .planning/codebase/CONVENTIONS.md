# Coding Conventions

**Analysis Date:** 2025-03-26

## Naming Patterns

**Files:**
- Python source files: `snake_case.py` (e.g., `rewards.py`, `track_manager.py`).
- Test files: `test_snake_case.py` (e.g., `test_track_manager.py`).
- Configuration files: `arcpro_env_cfg.py`, `arcpro_robot_cfg.py`.

**Functions/Methods:**
- `snake_case` (e.g., `speed_reward`, `get_telemetry_vector`, `sample_waypoints_from_usd`).
- Callback functions for Isaac Lab: `func=mdp_obs.get_telemetry_vector`.

**Variables:**
- `snake_case` (e.g., `lat_err`, `head_err`, `_TRACK_MANAGER` - global singletons use `UPPER_SNAKE_CASE` or `_PRIVATE_UPPER_SNAKE_CASE`).
- Tensor variables: Often use descriptive names or abbreviations like `pos`, `vel`, `q`, `jv`.

**Types:**
- Classes: `PascalCase` (e.g., `TrackManager`, `PolicyWrapper`, `ARCProEnvCfg`).
- Configuration classes: `@configclass` from `isaaclab.utils` for hierarchical configs.

## Code Style

**Formatting:**
- Indentation: 4 spaces.
- Adherence to PEP 8 guidelines is expected. `flake8` is listed in `requirements.txt`.
- Trailing commas are often used in multi-line lists/dictionaries for cleaner diffs.

**Linting:**
- Tool used: `flake8` (indicated by `requirements.txt`).
- Key rules: Standard PEP 8 rules for readability and consistency.

## Import Organization

**Order:**
1. Standard library imports (e.g., `os`, `sys`, `argparse`).
2. Third-party imports (e.g., `torch`, `numpy`, `isaaclab`, `omni`, `pxr`).
3. Local project imports (e.g., `from mdp.track_manager import TrackManager`).

**Path Aliases:**
- `sys.path.append(os.path.dirname(os.path.abspath(__file__)))` in configuration files to facilitate sibling imports.
- `sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "arcproLab"))` in test files.
- Manual `sys.path` modification in scripts to ensure root and package directories are available (e.g., `arcproLab/scripts/verify_metric.py`).

## Error Handling

**Patterns:**
- Explicit `try...except` blocks for external dependencies and optional features (e.g., `TrackManager` sampling from USD).
- Specific exception handling: `FileNotFoundError`, `RuntimeError`, or general `Exception as e`.
- NaN handling: Explicitly checking and replacing NaNs with zeros in reward and observation functions (e.g., `arcproLab/mdp/rewards.py`).
- Fallback logic: Providing default values or simplified behaviors if a resource (like a waypoint file) is missing.

## Logging

**Framework:**
- Standard `print()` statements with component-specific prefixes (e.g., `[TrackManager] Loaded ...`).

**Patterns:**
- Informational logging during initialization and loading phases.
- Conditional printing in simulation loops (e.g., `if count % 20 == 0:`) to avoid flooding the console.

## Comments

**When to Comment:**
- Docstrings (triple double quotes) for modules, classes, and functions.
- "Args" and "Returns" sections in docstrings for methods with multiple parameters or specific return types.
- Inline comments to explain complex mathematical logic, coordinate system transforms, or indices in tensors (e.g., observation vector mapping).
- Copyright and license headers in every file.

**JSDoc/TSDoc:**
- Not applicable (Python project).

## Function Design

**Size:**
- Single-responsibility functions. Complex logic is broken down into smaller helper methods (e.g., `TrackManager` splitting loading, sampling, and computation).

**Parameters:**
- Type hints are consistently used for function parameters.
- Default values are provided for optional parameters (e.g., `device: str = "cuda:0"`).
- Manager-based RL functions (rewards, observations) take `env: ManagerBasedRLEnv` as the primary argument.

**Return Values:**
- Type hints for return values.
- `torch.Tensor` is standard for data-heavy operations.

## Module Design

**Exports:**
- Modules export functions and classes directly.
- Singletons: `_TRACK_MANAGER` with `get_track_manager()` accessor.

**Barrel Files:**
- `__init__.py` files used for marking package directories (`mdp/`).

## Isaac Lab Patterns

**Configuration:**
- Hierarchical configs using `@configclass`.
- Separation of Scene, Observation, Action, Reward, and Termination configurations.
- `__post_init__` for late-binding configuration adjustments.

**Scripts:**
- Use of `AppLauncher` for launching Isaac Sim.
- `argparse` for CLI arguments, including `AppLauncher.add_app_launcher_args(parser)`.
- Conditional imports based on headless mode.

## Stability and Physics

**Standard PhysX Settings:**
- Use TGS (Temporal Gauss-Seidel) solver: `solver_type=1` in `PhysxCfg`.
- High precision for robots: `solver_position_iteration_count=32`, `solver_velocity_iteration_count=16` in `ArticulationRootPropertiesCfg`.
- Enable standard features: `enable_ccd=True`, `enable_stabilization=True`.
- **No stability workarounds:** Artificial friction, mass overrides, or root fixing (unless static) are avoided in favor of high-fidelity physical parameters.
- **1.0x Scaling:** All assets must use real-world 1.0x metric scale (`scale=(1.0, 1.0, 1.0)`) to ensure gravity and inertia are calculated correctly by PhysX.

---

*Convention analysis: 2025-03-26*
