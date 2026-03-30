# Testing Patterns

**Analysis Date:** 2025-03-26

## Test Framework

**Runner:**
- `pytest`
- Configuration: Standard `pytest` invocation.

**Assertion Library:**
- Standard Python `assert` statements.
- `torch.allclose()` or `torch.all()` for tensor-based numerical and boolean comparisons.
- `np.allclose()` or `np.array_equal()` for NumPy-based comparisons.

**Run Commands:**
```bash
pytest                 # Run all logic tests (mocked)
./run_gui_verify.sh    # Run simulation-based verification with GUI
./verify_sim.sh        # Run headless simulation verification
```

## Test File Organization

**Location:**
- Dedicated `tests/` directory at the project root for unit tests.
- Standalone verification scripts in `arcproLab/scripts/`.

**Naming:**
- Unit test files: `test_*.py` (e.g., `tests/test_track_manager.py`).
- Verification scripts: `verify_*.py` (e.g., `arcproLab/scripts/verify_metric.py`).

**Structure:**
```
[project-root]/
├── tests/
│   └── test_*.py      # Logic-only tests (fast, no Isaac Sim required)
└── arcproLab/
    └── scripts/
        └── verify_*.py # Simulation-based verification (requires Isaac Sim)
```

## Test Structure

**Suite Organization (Mocked):**
```python
import pytest
import torch
import sys
from unittest.mock import MagicMock

# Environment setup: Mock Isaac Sim before importing core components
sys.modules["omni"] = MagicMock()
sys.modules["omni.usd"] = MagicMock()
sys.modules["pxr"] = MagicMock()

@pytest.fixture
def mock_component():
    # Setup: Initialize component with mocks
    component = MyComponent(device="cpu")
    yield component
```

**Suite Organization (Simulation):**
```python
# Entry point for Isaac Sim verification scripts
from isaaclab.app import AppLauncher
app_launcher = AppLauncher(headless=True)
simulation_app = app_launcher.app

import torch
from arcproLab.arcpro_env_cfg import ARCProEnvCfg

def main():
    # Setup environment
    env_cfg = ARCProEnvCfg()
    # Run loop
    while simulation_app.is_running():
        # Step and assert
        ...
```

**Patterns:**
- **Fixture-based Setup:** Using `@pytest.fixture` to initialize objects and provide synthetic test data.
- **Dependency Mocking:** Injecting mocks into `sys.modules` to decouple tests from the heavy Isaac Sim environment.
- **Verification Scripts:** Comprehensive scripts that run the full simulation loop to verify physics stability and reward metrics.

## Mocking

**Framework:** `unittest.mock.MagicMock` (standard library).

**Patterns:**
- Mocking `omni`, `omni.usd`, and `pxr` allows running unit tests without a GPU or an Isaac Sim installation.
- Injecting synthetic data (e.g., waypoints) into components like `TrackManager` for isolated testing.

**What to Mock:**
- External simulation libraries.
- GPU-dependent operations in unit tests.

**What NOT to Mock:**
- Core mathematical logic.
- Observation and reward calculations that rely on tensor operations.

## Fixtures and Factories

**Test Data:**
- Manual creation of small, predictable tensors for input/output verification.
- Synthetic waypoints for track testing.

**Location:**
- Defined within the relevant `test_*.py` files or in a common `conftest.py` if shared.

## Coverage

**Requirements:** None explicitly enforced, but critical logic paths (e.g., `TrackManager`) are prioritized for unit testing.

**View Coverage:**
- Standard `pytest-cov` can be used.

## Test Types

**Unit Tests:**
- **Scope:** Mathematical and logical functions (e.g., distance calculations, error metrics).
- **Files:** `tests/test_track_manager.py`.

**Integration Tests:**
- **Scope:** Interaction between multiple components, though often with mocked environment.
- **Files:** Verification scripts like `arcproLab/scripts/verify_policy.py`.

**E2E / Simulation Verification:**
- **Scope:** Full-loop simulation runs to verify policy behavior in the USD environment.
- **Files:** `verify_sim.sh`, `arcproLab/scripts/verify_metric.py`.

## Common Patterns

**Async Testing:**
- None in standard unit tests. Isaac Sim scripts use synchronous loops (with `simulation_app.update()`).

**Error Testing:**
- Typically handled via `try...except` in the source code rather than explicit test suites.

**Physics Verification:**
- Using `verify_metric.py` to ensure the robot falls naturally and tracks its own pose correctly under standard PhysX settings.

---

*Testing analysis: 2025-03-26*
