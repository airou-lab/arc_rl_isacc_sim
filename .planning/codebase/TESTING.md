# Testing Patterns

**Analysis Date:** 2024-05-24

## Test Framework

**Runner:**
- `pytest` (specified in `requirements.txt`)
- Config: No separate `pytest.ini` found, uses default configuration.

**Assertion Library:**
- Native `assert` statements used with `pytest`.
- `torch.allclose()` for tensor comparisons.

**Run Commands:**
```bash
pytest                  # Run all tests
pytest tests/           # Run tests in specific folder
```

## Test File Organization

**Location:**
- Separate `tests/` directory at the project root.

**Naming:**
- `test_*.py` for test files. Example: `tests/test_track_manager.py`.

**Structure:**
```
[project-root]/
└── tests/
    └── test_track_manager.py
```

## Test Structure

**Suite Organization:**
```python
import pytest
from unittest.mock import MagicMock

# Local fixture with dependency mocking
@pytest.fixture
def mock_track_manager():
    # Mock complex external dependencies
    sys.modules["omni"] = MagicMock()
    # Setup
    tm = TrackManager(device="cpu")
    # Custom test data
    tm.waypoints = torch.tensor(...)
    return tm

# Individual test functions
def test_function_name(mock_track_manager):
    # Act
    result = mock_track_manager.some_method(...)
    # Assert
    assert result == expected
```

**Patterns:**
- Mocking of Isaac Sim/Omniverse modules (`omni`, `pxr`) to allow tests to run without a live simulation context.
- `sys.path.insert(0, ...)` used to ensure modules can be imported from parent directories during tests.

## Mocking

**Framework:** `unittest.mock` (standard library).

**Patterns:**
```python
# Mocking Isaac Sim/Omniverse modules
sys.modules["omni"] = MagicMock()
sys.modules["omni.usd"] = MagicMock()
sys.modules["pxr"] = MagicMock()
```

**What to Mock:**
- External simulation libraries that require a running GPU instance (`omni`, `pxr`).
- Hardware-specific interfaces or heavy file system access if not needed for the unit test.

**What NOT to Mock:**
- Core mathematical and logic modules (`numpy`, `torch`).
- The specific unit under test (e.g., `TrackManager`'s internal logic).

## Fixtures and Factories

**Test Data:**
```python
# Synthetic waypoints for testing
wps = np.zeros((10, 3))
wps[:, 0] = np.linspace(0, 9, 10)
tm.waypoints = torch.tensor(wps, device="cpu", dtype=torch.float32)
```

**Location:**
- Fixtures are currently defined within the test file itself.

## Coverage

**Requirements:** None enforced.

**View Coverage:**
Not configured (no `pytest-cov` detected in `requirements.txt`).

## Test Types

**Unit Tests:**
- Focus on logical calculations in isolated modules (e.g., `TrackManager`'s error calculations).
- Run on CPU to avoid simulation overhead and GPU requirements.

**Integration Tests:**
- Not explicitly detected, though Isaac Sim typically uses its own test runner for integrated simulation tests.

**E2E Tests:**
- Not used.

## Common Patterns

**Async Testing:**
- Not yet present in unit tests, but common in live simulation testing.

**Error Testing:**
- Use `pytest.raises()` (not seen in existing tests, but standard for the framework).

---

*Testing analysis: 2024-05-24*
