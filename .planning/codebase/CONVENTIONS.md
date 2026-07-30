# Coding Conventions

**Analysis Date:** 2026-07-28

## Naming Patterns

**Files:**
- snake_case for Python modules (`arcpro_env_cfg.py`, `test_track_bounds.py`)
- UPPERCASE for configuration and constants (e.g., `ARCPRO_ROBOT_CFG`)

**Functions:**
- snake_case for functions (e.g., `waypoint_progress_reward`, `reset_robot_to_fixed_spawn`)

**Variables:**
- snake_case for variables (e.g., `env_cfg`, `simulation_app`)
- UPPER_SNAKE_CASE for global constants (e.g., `ARCPRO_LAB_DIR`, `USD_DIR`)

**Types:**
- PascalCase for classes (e.g., `ARCProSceneCfg`, `ARCProEnvCfg`, `EventCfg`)

## Code Style

**Formatting:**
- No strict formatter like `black` or `yapf` was detected.
- Code relies on manual formatting, typical 4-space indentation.

**Linting:**
- `flake8` is used in the CI pipeline (`.github/workflows/ci.yml`).
- Syntax errors and undefined names are treated as errors (`--select=E9,F63,F7,F82`).
- A max complexity of 10 and max line length of 127 are checked but exit as warnings (`--exit-zero`).

## Import Organization

**Order:**
1. Standard library imports (`os`, `sys`, `math`, `argparse`)
2. Path manipulations (e.g., `sys.path.append(...)`)
3. Third-party library imports (`torch`, `numpy`)
4. Isaac Sim / IsaacLab imports (`isaaclab.app`, `isaaclab.utils`, `omni.usd`, `pxr`)
5. Local project imports (`arcpro_robot_cfg`, `mdp.*`)

**Path Aliases:**
- Direct `sys.path.append(os.path.dirname(os.path.abspath(__file__)))` is heavily used to allow local module resolution.

## Error Handling

**Patterns:**
- Minimal explicit error handling in simulation scripts. 
- Some test scripts use `try/except Exception as e:` blocks with a `print` statement to catch issues without crashing (e.g., `test_torch.py`).

## Logging

**Framework:** `console` (print statements)

**Patterns:**
- Prints are heavily used to display debugging output, tensor shapes, and manually verified values in test scripts (e.g., `print(f"RAYCAST HIT: {hit['hit']}")`).
- The training process logs metrics using TensorBoard natively through `skrl` (`train_skrl.py`).

## Comments

**When to Comment:**
- Used heavily for configuration documentation, especially tuning notes inside `arcpro_env_cfg.py`.
- Inline comments explain specific math formulas, physics properties, and historical tuning context.

**JSDoc/TSDoc / Python Docstrings:**
- Docstrings (`"""..."""`) are used minimally, mostly on classes (e.g., `"""Configuration for events."""`).

## Function Design

**Size:** Moderate to Large (specifically configuration classes like `ARCProSceneCfg` which can grow very long).

**Parameters:** Heavy use of named parameters and dataclass structures via `@configclass` from IsaacLab.

**Return Values:** Functions typically return tensors (in RL reward functions) or `None` if they modify state.

## Module Design

**Exports:** No strict `__all__` definitions observed. Imports are direct.

**Barrel Files:** `__init__.py` in `arcproLab` is used as a Gym environment registration barrel file (e.g., registering `ARCPro-RL-v1`).

---

*Convention analysis: 2026-07-28*
