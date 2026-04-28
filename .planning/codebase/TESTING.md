# Testing Patterns

**Analysis Date:** 2024-04-23

## Test Framework

**Runner:**
- `pytest` for unit tests.
- Manual execution for simulation verification scripts.

**Assertion Library:**
- `assert` (standard Python).
- `torch.testing` for tensor comparisons.

**Run Commands:**
```bash
pytest tests/                          # Run unit tests
./arcproLab/scripts/verify_metric.sh   # Verify sim scaling
./run_gui_verify.sh                    # Run GUI-based visual audit
```

## Test File Organization

**Location:**
- Unit tests: `tests/` directory.
- Verification scripts: `arcproLab/scripts/` (for simulation-in-the-loop tests).

**Naming:**
- Unit tests: `test_*.py`.
- Verification scripts: `verify_*.py` or `audit_*.py`.

## Test Structure

**Verification Script Pattern:**
```python
# arcproLab/scripts/verify_metric.py
def main():
    # 1. Launch App
    # 2. Setup Env
    # 3. Step Simulation
    # 4. Assert Geometric Properties (e.g., track width == 3.5m)
```

**Patterns:**
- **Scene Audit:** Scripts that spawn the environment and print dimensions to verify USD scaling.
- **Physics Audit:** Scripts that apply forces and measure resulting velocity to verify mass/friction.

## Mocking

**Framework:** None observed; Tests typically use the live Isaac Sim environment or simple NumPy/Torch mocks for math.

**What to Mock:**
- Environment state in `TrackManager` unit tests.

**What NOT to Mock:**
- Physics interactions (always use Isaac Sim for these).

## Fixtures and Factories

**Test Data:**
- `track_centerline_1x.npy`: Reference path for `TrackManager` tests.

**Location:**
- `arcproLab/mdp/` (for tracking data).

## Coverage

**Requirements:** No formal coverage target enforced.

**View Coverage:**
- Not configured.

## Test Types

**Unit Tests:**
- Geometric calculations in `TrackManager`.
- Reward function logic (isolated from sim).

**Verification Tests (System):**
- **Spawn Verification:** Ensures robot lands on the track without falling through.
- **Marker Verification:** Ensures boundary markers align with visual lines.
- **Metric Verification:** Ensures 1 unit in Sim == 1 meter in reality.

**Policy Verification:**
- Inference tests using `verify_policy.py` to check for model performance and regressions.

## Common Patterns

**Async Testing:** Not used (Simulation is synchronous).

**Error Testing:**
- Manual checks for NaNs in logs.
- Unit tests for edge cases in geometric calculations (e.g., division by zero in curvature).

---

*Testing analysis: 2024-04-23*
