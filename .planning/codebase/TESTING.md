# Testing Patterns

**Analysis Date:** 2026-07-28

## Test Framework

**Runner:**
- Custom standalone Python scripts (No `pytest` runner is actively used, though CI tries to run `pytest tests/`, the directory does not exist).
- Standalone test scripts use `isaaclab.app.AppLauncher` to start Isaac Sim manually.

**Assertion Library:**
- None detected. Assertions are typically manual (printing values to console and visually inspecting results).

**Run Commands:**
```bash
python test_raycast.py              # Test raycasting manually
python test_track_bounds.py         # Test track bounds manually
python arcproLab/test_rewards.py    # Test reward step execution manually
```

## Test File Organization

**Location:**
- Co-located in the project root (`test_*.py`) and some inside modules (`arcproLab/test_rewards.py`).

**Naming:**
- `test_*.py` pattern is used (e.g., `test_raycast.py`, `test_track_bounds.py`, `test_torch.py`).

**Structure:**
```
[project-root]/
├── test_joints.py
├── test_obs.py
├── test_raycast.py
└── test_track_bounds.py
```

## Test Structure

**Suite Organization:**
```python
import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

# Import modules and create environment
from arcproLab.arcpro_env_cfg import ARCProEnvCfg
# ... environment setup ...

# Manual execution and printing
# ...
print("Results: ...")

# Close app
simulation_app.close()
```

**Patterns:**
- Setup: Initialize `AppLauncher`, parse arguments, setup Isaac Sim `simulation_app`, initialize `ManagerBasedEnv`.
- Teardown: `simulation_app.close()` at the end of the script.
- Assertion pattern: Run `env.step(action)` or physics queries, then `print()` the results for visual inspection.

## Mocking

**Framework:** None

**Patterns:**
```python
# No mocking used. Simulation tests run full physics engine logic.
```

**What to Mock:**
- None. Testing is done via integration with the full physics simulator.

**What NOT to Mock:**
- Do not mock `omni.physx` or `isaaclab` core components. The scripts are meant to test realistic physics behaviors (e.g. raycasts, bounds).

## Fixtures and Factories

**Test Data:**
```python
actions = torch.zeros((1, 2), device="cuda:0")
actions[:, 0] = 0.0 # Steer straight
actions[:, 1] = -1.0 # Brake
```
- Data is manually constructed via PyTorch tensors inline.

**Location:**
- Inline inside individual test scripts.

## Coverage

**Requirements:** None enforced

**View Coverage:**
```bash
# Not applicable
```

## Test Types

**Unit Tests:**
- Not used. Simulating individual components without the Isaac environment is avoided.

**Integration Tests:**
- All `test_*.py` files act as integration tests, requiring a full Isaac Sim instance to be booted up.

**E2E Tests:**
- Evaluated via standalone playing scripts (`play_skrl.py` or `debug.sh`) outside of the automated test structure.

## Common Patterns

**Async Testing:**
```python
# Not used. The tests are synchronous simulation loops.
```

**Error Testing:**
```python
try:
    reward_buf += val
    print("NO CRASH!")
except Exception as e:
    print("CRASH:", e)
```

---

*Testing analysis: 2026-07-28*
