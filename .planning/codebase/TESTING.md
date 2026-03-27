# Testing Patterns

**Analysis Date:** 2024-07-29

## Test Framework

**Runner:**
- `pytest`
- Config: Not explicitly defined (e.g., `pytest.ini`, `pyproject.toml`) but invoked directly.

**Assertion Library:**
- Standard `assert` statements.
- `torch.allclose` for numerical comparisons of tensors.

**Run Commands:**
```bash
pytest                 # Run all tests
```

## Test File Organization

**Location:**
- Separate `tests/` directory at the project root.

**Naming:**
- Files are named `test_*.py` (e.g., `test_track_manager.py`).

**Structure:**
```
[project-root]/
├── tests/
│   └── test_module_name.py
└── arcproLab/
    └── module_name/
        └── ...
```

## Test Structure

**Suite Organization:**
```python
import pytest
# ... imports ...

@pytest.fixture
def my_fixture():
    # Setup code
    yield value
    # Teardown code

def test_something(my_fixture):
    # Test logic
    assert ...
```

**Patterns:**
- **Setup pattern:** `pytest.fixture` functions are used to prepare test environments and data. Fixtures can include mocking external dependencies.
- **Teardown pattern:** `yield` in `pytest.fixture` allows for teardown logic after the test or test suite finishes.
- **Assertion pattern:** Standard Python `assert` keyword is used for boolean checks. `torch.allclose` is used for floating-point tensor comparisons.

## Mocking

**Framework:** `unittest.mock.MagicMock` (from Python's standard library).

**Patterns:**
```python
import sys
from unittest.mock import MagicMock

@pytest.fixture
def mock_external_dependencies():
    sys.modules["omni"] = MagicMock()
    sys.modules["omni.usd"] = MagicMock()
    sys.modules["pxr"] = MagicMock()
    # ... then initialize objects that depend on these mocks ...
```

**What to Mock:**
- External simulation-specific libraries (`omni`, `pxr`) that are not available or desirable in a pure Python testing environment.

**What NOT to Mock:**
- Core business logic or internal dependencies that should be tested directly.

## Fixtures and Factories

**Test Data:**
```python
# Inside a pytest fixture
wps = np.zeros((10, 3))
wps[:, 0] = np.linspace(0, 9, 10)
wps[:, 2] = 0.0 # Facing +X
self.waypoints = torch.tensor(wps, device="cpu", dtype=torch.float32)
```

**Location:**
- Defined as `pytest.fixture` functions within the test files (`test_*.py`).

## Coverage

**Requirements:** None explicitly enforced.

**View Coverage:**
- Not observed, but `pytest-cov` is a common plugin for `pytest` to generate coverage reports.

## Test Types

**Unit Tests:**
- **Scope and approach:** Focus on individual functions and methods (e.g., `TrackManager` methods). Mocks are used to isolate the unit under test from external dependencies.

**Integration Tests:**
- Not explicitly identified as a separate category, but tests involving multiple components of `TrackManager` and its interactions (even with mocked external systems) could be considered integration-like.

**E2E Tests:**
- Not used.

## Common Patterns

**Async Testing:**
- Not applicable (no asynchronous code observed in tested units).

**Error Testing:**
- Not explicitly observed in the provided sample, but typically involves using `pytest.raises` for expected exceptions.

---

*Testing analysis: 2024-07-29*
