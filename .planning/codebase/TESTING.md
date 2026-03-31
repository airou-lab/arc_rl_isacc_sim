# Testing Patterns

**Analysis Date:** 2024-05-21

## Test Framework

**Runner:**
- `pytest`
- `isaaclab.sh` is used as a wrapper to run Isaac Sim scripts.

**Assertion Library:**
- `pytest` (standard `assert` statements)
- `torch.allclose` for numerical verification in physics tests.

**Run Commands:**
```bash
# Run unit tests
pytest tests/

# Run physics and spawn verification (headless)
./verify_sim.sh

# Run metric accuracy verification (headless)
./verify_sim_metric.sh

# Run visual policy verification (GUI)
./run_gui_verify.sh
```

## Test File Organization

**Location:**
- Unit tests are located in `tests/`.
- Verification and sanity check scripts are in `arcproLab/scripts/`.

**Naming:**
- Unit tests: `test_*.py`.
- Verification scripts: `verify_*.py`.

## Test Structure

**Suite Organization:**
```python
# tests/test_track_manager.py
@pytest.fixture
def mock_track_manager():
    # Mock omni and pxr before creating TrackManager
    sys.modules["omni"] = MagicMock()
    # ...
    tm = TrackManager(device="cpu")
    # ...
    return tm

def test_closest_waypoint(mock_track_manager):
    # Setup
    pos = torch.tensor([[2.1, 0.5, 0.0]], device="cpu")
    # Action
    closest = mock_track_manager.get_closest_waypoint_data(pos)
    # Assertion
    assert closest[0, 0] == 2.0
```

## Mocking

**Framework:** `unittest.mock.MagicMock`

**Patterns:**
```python
# Mocking Isaac Sim dependencies for pure unit tests
sys.modules["omni"] = MagicMock()
sys.modules["omni.usd"] = MagicMock()
sys.modules["pxr"] = MagicMock()
```

**What to Mock:**
- Isaac Sim modules (`omni`, `pxr`) when running in a standard Python environment (outside `isaaclab.sh`).
- GPU/Cuda operations when testing on CPU-only runners.

## Verification Workflows

### 1. Physics & Spawn Verification (`verify_spawn.py`)
- Purpose: Sanity check to ensure the robot drops correctly onto the track and the environment initializes without errors.
- Workflow:
    1. Spawn environment using `ARCProEnvCfg`.
    2. Check initial robot position and error calculations.
    3. Run 50 steps of simulation with zero actions.
    4. Log positions and termination status.

### 2. Metric Accuracy Verification (`verify_metric.py`)
- Purpose: Ensure the 1.0x metric scaling is correctly applied and telemetry reports realistic values.
- Workflow:
    1. Spawn environment in headless mode.
    2. Drive the robot forward at a target velocity (e.g., 40 rad/s ~ 2.0 m/s).
    3. Audit joint velocities and lateral errors.

### 3. Visual Policy Verification (`run_gui_verify.sh`)
- Purpose: Visual inspection of the robot's behavior with the SB3 policy enabled.
- Workflow:
    1. Launch `verify_policy.py` with `--enable_cameras` and GUI.
    2. Display real-time telemetry in a `TelemetryWindow`.
    3. Observe lane following and obstacle avoidance behavior.

## Common Patterns

**Async Testing:**
- Simulation steps are synchronous but use `torch.no_grad()` to avoid overhead during verification:
  ```python
  with torch.no_grad():
      obs, rewards, terminations, truncations, extras = env.step(actions)
  ```

**Error Testing:**
- Verify error calculation logic in `TrackManager` using synthetic waypoints and known robot positions:
  ```python
  lat_err, head_err = tm.compute_errors(pos, yaw)
  assert torch.allclose(lat_err, expected_val)
  ```

---

*Testing analysis: 2024-05-21*
