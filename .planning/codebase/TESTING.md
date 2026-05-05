# Testing Patterns

**Analysis Date:** 2025-01-23

## Test Framework

**Runner:**
- `pytest` (Configured in `arcproLab/policy_stack/pytest.ini`)
- Custom smoke test runner: `arcproLab/policy_stack/test_all_so_far.py`

**Assertion Library:**
- Standard Python `assert` statements.
- `torch.allclose()` and `np.allclose()` for numerical/tensor comparisons.

**Run Commands:**
```bash
pytest tests/                          # Run standard pytest suite
python arcproLab/policy_stack/test_all_so_far.py  # Run comprehensive smoke test suite
```

## Test File Organization

**Location:**
- Separate `tests/` directory (e.g., `tests/test_track_manager.py`).
- Integrated tests within subpackages (e.g., `arcproLab/policy_stack/test_all_so_far.py`).

**Naming:**
- `test_*.py` for standard tests.
- Scripts prefixed with `test_` in `arcproLab/scripts/` (e.g., `test_sb3_wrapper.py`).

**Structure:**
```
[project-root]/
├── tests/                 # Dedicated test directory
└── arcproLab/
    ├── policy_stack/
    │   └── test_all_so_far.py # Monolithic smoke test
    └── scripts/
        └── test_*.py      # Utility/Verification scripts
```

## Test Structure

**Suite Organization:**
```python
# From tests/test_track_manager.py
@pytest.fixture
def mock_track_manager():
    # Setup
    tm = TrackManager(device="cpu")
    return tm

def test_feature(mock_track_manager):
    # Action
    result = mock_track_manager.do_something()
    # Assertion
    assert result == expected
```

**Patterns:**
- **Setup pattern:** Use `@pytest.fixture` for reusable components.
- **Teardown pattern:** Use `with tempfile.TemporaryDirectory()` for file-based tests.
- **Assertion pattern:** Direct equality for scalars, `allclose` for tensors/arrays.

## Mocking

**Framework:** `unittest.mock.MagicMock`

**Patterns:**
```python
# Mocking heavy dependencies to run tests without Isaac Sim
import sys
from unittest.mock import MagicMock
sys.modules["omni"] = MagicMock()
sys.modules["omni.usd"] = MagicMock()
sys.modules["pxr"] = MagicMock()
```

**What to Mock:**
- Isaac Sim / Omniverse modules (`omni`, `pxr`).
- Hardware/Environment interactions when testing core logic.

**What NOT to Mock:**
- Mathematical models (Ackermann, Planar planner).
- Configuration objects.

## Fixtures and Factories

**Test Data:**
```python
# Synthetic waypoints for testing TrackManager
wps = np.zeros((10, 3))
wps[:, 0] = np.linspace(0, 9, 10)
tm.waypoints = torch.tensor(wps)
```

**Location:**
- Defined within the test files as fixtures or helper functions (e.g., `_create_synthetic_dataset`).

## Coverage

**Requirements:** Not explicitly enforced via config, but `test_all_so_far.py` performs a "Syntax check all files" to ensure no major breakage.

## Test Types

**Unit Tests:**
- Mathematics and geometry (Ackermann, Planar planner).
- Configuration loading/saving.

**Integration Tests:**
- `WorkerNode` interaction with `IntersectionGraph`.
- `WorkerScheduler` conflict detection.

**Smoke Tests:**
- `test_all_so_far.py` runs a "Mini training loop" to verify the whole stack.

**Syntax Checks:**
- Automated `ast.parse()` check on all critical files in `test_all_so_far.py`.

## Common Patterns

**Async Testing:**
- Not observed; Isaac Sim tests generally wait for the simulation step to complete.

**Error Testing:**
- `pytest.raises` or `try-except` in custom runners to verify expected failures or handle environment-specific skips.

---

*Testing analysis: 2025-01-23*
