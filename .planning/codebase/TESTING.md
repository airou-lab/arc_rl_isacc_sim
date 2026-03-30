# Testing Patterns

**Analysis Date:** 2025-03-26

## Test Framework

**Runner:**
- `pytest`
- Configuration: No explicit `pytest.ini` or `pyproject.toml` found; typically invoked from the project root.

**Assertion Library:**
- Standard Python `assert` statements.
- `torch.allclose()` or `torch.all()` for tensor-based numerical and boolean comparisons.
- `np.allclose()` or `np.array_equal()` for NumPy-based comparisons.

**Run Commands:**
```bash
pytest                 # Run all tests
pytest tests/          # Explicitly run tests in the tests directory
```

## Test File Organization

**Location:**
- Dedicated `tests/` directory at the project root for unit tests.
- Tool-specific verification scripts (standalone tests) in `trash/tools/` or `arcproLab/scripts/`.

**Naming:**
- Unit test files: `test_*.py` (e.g., `tests/test_track_manager.py`).
- Verification scripts: `verify_*.py`, `test_*.py` (e.g., `trash/tools/test_imports.py`).

**Structure:**
```
[project-root]/
├── tests/
│   └── test_*.py
└── arcproLab/
    └── [source-files].py
```

## Test Structure

**Suite Organization:**
```python
import pytest
import torch
import sys
from unittest.mock import MagicMock

# Environment setup
sys.modules["omni"] = MagicMock()
sys.modules["omni.usd"] = MagicMock()
sys.modules["pxr"] = MagicMock()

@pytest.fixture
def mock_component():
    # Setup: Initialize component with mocks
    component = MyComponent(device="cpu")
    yield component
    # Teardown (if any)

def test_feature_a(mock_component):
    # Test logic
    result = mock_component.run()
    assert result == expected_value
```

**Patterns:**
- **Fixture-based Setup:** Using `@pytest.fixture` to initialize objects and provide synthetic test data.
- **Dependency Mocking:** Injecting mocks into `sys.modules` to decouple tests from the heavy Isaac Sim / Omniverse environment.
- **Data-driven Assertions:** Comparing tensor outputs against hand-calculated or expected baseline tensors.

## Mocking

**Framework:** `unittest.mock.MagicMock` (standard library).

**Patterns:**
```python
@pytest.fixture
def mock_track_manager():
    # Mock omni and pxr before creating TrackManager
    sys.modules["omni"] = MagicMock()
    sys.modules["omni.usd"] = MagicMock()
    sys.modules["pxr"] = MagicMock()
    
    tm = TrackManager(device="cpu")
    # Synthetic waypoints injection
    tm.waypoints = torch.tensor(...)
    return tm
```

**What to Mock:**
- Isaac Sim / Omniverse libraries (`omni`, `pxr`).
- Hardware-dependent or simulation-dependent calls that shouldn't run in a CI/CD environment.

**What NOT to Mock:**
- Pure logic classes (e.g., `TrackManager`'s mathematical computations).
- Data processing pipelines (`PolicyWrapper`'s tensor transformations).

## Fixtures and Factories

**Test Data:**
- Manual creation of small, predictable tensors for input/output verification.
- Synthetic waypoints for track testing.

**Location:**
- Defined within the relevant `test_*.py` files.

## Coverage

**Requirements:** None explicitly defined.

**View Coverage:**
- Not explicitly configured, but `pytest-cov` is recommended.

## Test Types

**Unit Tests:**
- **Scope:** Mathematical and logical functions (e.g., distance calculations, error metrics).
- **Files:** `tests/test_track_manager.py`.

**Integration Tests:**
- **Scope:** Interaction between multiple components, though often with mocked environment.
- **Files:** Verification scripts that run within Isaac Sim (e.g., `arcproLab/scripts/verify_policy.py`).

**E2E / Simulation Verification:**
- **Scope:** Full-loop simulation runs to verify policy behavior in the USD environment.
- **Files:** `verify_sim.sh`, `arcproLab/scripts/verify_metric.py`.

## Common Patterns

**Async Testing:**
- None in standard unit tests. Isaac Sim scripts use synchronous loops (with `simulation_app.update()`).

**Error Testing:**
- Typically handled via `try...except` in the source code rather than explicit test suites.

---

*Testing analysis: 2025-03-26*
