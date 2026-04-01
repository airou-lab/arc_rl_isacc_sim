# Testing Patterns

**Analysis Date:** 2024-10-24

## Test Framework

**Runner:**
- pytest (Version 7.0+)
- Config: None (standard discovery)

**Assertion Library:**
- Standard `assert` in Python.

**Run Commands:**
```bash
pytest                  # Run unit tests in tests/
./run_gui_verify.sh     # Run visual verification in Isaac Sim
./verify_sim.sh         # Run headless simulation verification
python3 arcproLab/scripts/verify_policy.py --checkpoint models/road_following_model.pth
```

## Test File Organization

**Location:**
- Logic tests: `tests/` directory.
- Environment verification: `arcproLab/scripts/` (e.g., `verify_metric.py`).

**Naming:**
- `test_*.py` for unit tests.
- `verify_*.py` for simulation-based verification.

**Structure:**
```
tests/
  └── test_track_manager.py
```

## Test Structure

**Suite Organization (12-Float Telemetry Verification):**
```python
def test_get_telemetry_vector(env):
    obs = get_telemetry_vector(env)
    assert obs.shape[1] == 12
    assert obs[:, 3] > 0 # Speed should be positive when moving
```

**Patterns:**
- **Unit Testing**: Standard pytest for logic without the full simulator.
- **Metric Verification**: Use of dedicated scripts like `verify_metric.py` to check physics (e.g., **0.5x robot scale**, **3.5kg mass**, settled Z position) within a live simulation.

## Mocking

**Framework:** `unittest.mock` (standard) or simple NumPy-based mocking of data.

**What to Mock:**
- **Absolute Waypoint data** for `TrackManager`.
- Robot state vectors for observation logic tests.

**What NOT to Mock:**
- Isaac Sim engine (use live verification instead).

## Fixtures and Factories

**Test Data:**
- `track_centerline.npy` used as the primary source for navigation tests.

**Location:**
- `arcproLab/mdp/track_centerline.npy`.

## Coverage

**Requirements:** None enforced.

**View Coverage:**
```bash
pytest --cov=arcproLab
```

## Test Types

**Unit Tests:**
- Tests for navigation math and track tracking logic in `tests/`.

**Integration Tests:**
- Verification scripts that launch Isaac Sim to check asset loading and physical interactions at **0.5x scale**.

**E2E Tests (Visual):**
- Scripts like `verify_policy.py` that allow a human to observe the robot's performance using the **12-float protocol UI**.

## Common Patterns

**Async Testing:** Not used (standard for synchronous simulation steps).

**Error Testing:**
- Use of `pytest.raises()` for expected errors.

---

*Testing analysis: 2024-10-24*
