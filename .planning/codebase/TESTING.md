# Testing Patterns

**Analysis Date:** 2025-04-18

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
python3 arcproLab/scripts/verify_policy.py --checkpoint arcproLab/models/road_following_model.pth
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

**Suite Organization (Telemetry Vector Verification):**
```python
def test_telemetry_normalization():
    # Test that 8.0x speed is normalized to 1.0x metric speed
    sim_speed = 8.0
    normalized_speed = sim_speed * 0.125
    assert normalized_speed == 1.0
```

**Patterns:**
- **Unit Testing**: Testing the math in `TrackManager` and `observations.py` logic without needing the simulator.
- **Metric Verification**: Live verification in Isaac Sim using `verify_metric.py` to check real-world scale and physics parameters (mass, friction, 4WD).

## Mocking

**Framework:** `unittest.mock` (standard).

**What to Mock:**
- **USD Stage**: When testing `TrackManager.sample_waypoints_from_usd` logic.
- **Robot Data**: Mocking `asset.data` from IsaacLab to test observation calculations.

**What NOT to Mock:**
- Physics interactions (use live verification instead).

## Fixtures and Factories

**Test Data:**
- `arcproLab/mdp/track_centerline.npy`: The source of truth for navigation tests.

## Coverage

**Requirements:** None enforced.

## Test Types

**Unit Tests:**
- Waypoint lookup and error calculation math in `TrackManager`.

**Integration Tests:**
- Asset loading and reference resolution in `openStreetUSD/`.
- Training initialization via `train_policy.py`.

**E2E Tests (Visual):**
- Full inference lap testing with `verify_policy.py`.
- Physics stability check under 4WD acceleration.

## Common Patterns

**Async Testing:** Not used (standard synchronous simulation).

**Error Testing:**
- Use of `pytest.raises()` for expected failures in waypoint loading.

---

*Testing analysis: 2025-04-18*
