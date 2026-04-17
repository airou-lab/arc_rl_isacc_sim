# Testing Patterns

**Analysis Date:** 2025-05-20

## Test Framework

**Runner:**
- `pytest` for unit tests.
- Custom verification scripts for simulation audits.

**Run Commands:**
```bash
pytest tests/                   # Run logic unit tests
./verify_sim.sh                 # Headless performance/metric audit
./run_gui_verify.sh             # Visual verification with Telemetry UI
python arcproLab/scripts/verify_spawn.py # Verify robot-track alignment
```

## Test File Organization

**Location:**
- Unit tests: `tests/`.
- Simulation tests/audits: `arcproLab/scripts/`.

**Naming:**
- `test_*.py` for logic validation.
- `verify_*.py` for simulation sanity checks.
- `audit_*.py` for deep-dive resource inspection (physics, mass, USD structure).

## Test Structure

**Logic Testing (Pytest):**
```python
def test_compute_errors():
    tm = TrackManager(device="cpu")
    # ... setup dummy waypoints ...
    lat_err, head_err = tm.compute_errors(pos, yaw)
    assert torch.abs(lat_err) < 0.01
```

**Simulation Verification Pattern:**
```python
# Launch Isaac Sim
app_launcher = AppLauncher(args_cli)
# Setup Env
env = ManagerBasedRLEnv(cfg=ARCProEnvCfg())
# Run steps and check state
obs, _ = env.reset()
for _ in range(100):
    env.step(zero_actions)
    # verify lat_err in env.extras
```

## Mocking

**Framework:**
- Minimal mocking. Most tests run either pure-math logic or full-physics simulation.

**What NOT to Mock:**
- Physics interactions (collisions, joint friction).
- USD stage traversal.

## Coverage

**Priority Areas:**
- `TrackManager` waypoint ordering and error math.
- `observations.py` telemetry vector indexing.
- `terminations.py` lane boundary thresholds.

## Test Types

**Unit Tests:**
- Waypoint sequence generation.
- Quaternion-to-Yaw conversion.

**Integration Tests:**
- Action scaling (Throttle 1.0 -> Joint Velocity 60.0).
- Reset Event success (Robot snaps to road height).

**Functional Audits:**
- `audit_live.py`: Checks for misaligned physics colliders.
- `verify_metric.py`: Validates that 1.0m in sim matches 1.0m in physics properties.

## Common Patterns

**Async Testing:**
- Simulation steps are blocking; no async/await patterns used in core MDP testing.

**Error Testing:**
- Intentionally place the robot outside the lane boundaries to verify that `white_line_contact` triggers a reset.
- Drive the robot in reverse to verify that `speed_reward` becomes negative.

---

*Testing analysis: 2025-05-20*
