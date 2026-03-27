# Coding Conventions

**Analysis Date:** 2024-07-29

## Naming Patterns

**Files:**
- Python source files: `snake_case.py` (e.g., `rewards.py`, `track_manager.py`).
- Test files: `test_snake_case.py` (e.g., `test_track_manager.py`).

**Functions:**
- `snake_case` (e.g., `speed_reward`, `get_telemetry_vector`, `sample_waypoints_from_usd`).

**Variables:**
- `snake_case` (e.g., `lat_err`, `head_err`, `_TRACK_MANAGER`).

**Types:**
- Classes: `CapitalCase` (e.g., `TrackManager`).

## Code Style

**Formatting:**
- Not explicitly configured by dedicated formatter, but `flake8` is listed in `requirements.txt` suggesting adherence to PEP 8.
- Indentation: 4 spaces.

**Linting:**
- Tool used: `flake8` (indicated by `requirements.txt`).
- Key rules: Adherence to PEP 8 guidelines is expected.

## Import Organization

**Order:**
1. Standard library imports (e.g., `os`, `sys`).
2. Third-party imports (e.g., `torch`, `numpy`, `isaaclab`, `omni`, `pxr`).
3. Local project imports (e.g., `from mdp.track_manager import TrackManager`).

**Path Aliases:**
- `sys.path.insert` is used in test files to make `arcproLab` discoverable for imports (e.g., `sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "arcproLab"))`).

## Error Handling

**Patterns:**
- Explicit `try...except` blocks are used for handling potential runtime issues, especially when dealing with external dependencies or optional features (e.g., `arcproLab/mdp/observations.py`, `arcproLab/mdp/track_manager.py`).
- NaNs are explicitly checked and handled, usually by replacing them with zeros (e.g., `arcproLab/mdp/rewards.py`, `arcproLab/mdp/observations.py`).

## Logging

**Framework:**
- Standard `print()` statements are used for informational messages and debugging, especially in `TrackManager` for feedback during waypoint loading/sampling.

**Patterns:**
- Informative messages typically prefixed with `[ComponentName]` (e.g., `[TrackManager] Loaded ...`).

## Comments

**When to Comment:**
- Docstrings are used for modules, classes, and functions to describe their purpose, arguments, and return values.
- Inline comments explain complex logic or specific indices/components of data structures (e.g., `Index 3: Forward Speed (m/s) - Local X velocity` in `arcproLab/mdp/observations.py`).
- Copyright and license information at the top of each file.

**JSDoc/TSDoc:**
- Not applicable (Python project).

## Function Design

**Size:**
- Functions generally focus on a single responsibility.
- No strict size limits observed, but functions are kept concise.

**Parameters:**
- Type hints are consistently used for function parameters.
- Default values are provided where appropriate.

**Return Values:**
- Type hints are consistently used for function return values.
- `torch.Tensor` is a common return type for functions dealing with numerical computations.

## Module Design

**Exports:**
- Modules export functions and classes directly.
- No explicit `__all__` declaration observed.

**Barrel Files:**
- `__init__.py` files are used in directories like `mdp/` to mark them as Python packages.

---

*Convention analysis: 2024-07-29*
