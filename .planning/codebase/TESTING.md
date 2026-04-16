# Testing Patterns

**Analysis Date:** 2025-05-15

## Test Framework

**Runner:**
- `pytest` for unit testing logic.
- Bash scripts for simulation-based verification (`verify_sim.sh`, `run_gui_verify.sh`).

**Run Commands:**
```bash
pytest tests/           # Run logic tests (TrackManager)
./verify_sim.sh         # Headless performance audit
./run_gui_verify.sh     # Visual verification of latest model
python arcproLab/scripts/verify_metric.py --num_envs 1 # Detailed metric and joint audit
```

## Test File Organization

**Location:**
- Logic tests are in `tests/`.
- Simulation verification scripts are in `arcproLab/scripts/`.

**Naming:**
- `test_*.py` for unit tests.
- `verify_*.py` for simulation sanity checks.

## Unit Testing Structure

**Logic Testing:**
- Focus on `TrackManager` and vectorized math in `mdp/`.
- Verifies waypoint generation, distance calculations, and coordinate transformations at 1x scale.

## Simulation Verification Patterns

**1x Metric Verification Loop:**
- **Metric Integrity**: Run `verify_metric.py` to ensure positions, velocities, and joint efforts align with metric expectations (e.g., speed in m/s, mass in kg).
- **Spawn & Reset Audit**: `verify_spawn.py` ensures the robot snaps correctly to the road and starts in the correct orientation.
- **Visual & FOV Audit**: `run_gui_verify.sh` allows manual verification of the camera FOV and the `fov_visibility_termination` logic.
- **Telemetry Audit**: Use the Telemetry UI (`mdp/visual_analytics.py`) to observe real-time speed, lateral error, and steering response.

## Mocking

**Framework:**
- Simulation verification relies on the live Isaac Sim environment.
- No extensive mocking of physics components is used.

## Coverage

**Requirements:**
- High coverage for `TrackManager` and `observations.py` logic.

## Test Types

**Unit Tests:**
- Waypoint error calculations (`tests/test_track_manager.py`).
- Telemetry vector construction shape and range checks.

**Integration Tests:**
- Robot-track alignment during reset events.
- Action-to-Joint velocity mapping (e.g., 60.0 throttle -> ~3.0 m/s).

**E2E Tests:**
- Full training-to-verification pipeline using `train.sh` followed by `run_gui_verify.sh`.

## Common Patterns

**Termination Testing:**
- Intentional driving off-road or out-of-FOV to verify that the environment resets and prints the correct termination reason to stdout.

**Joint Audit:**
- `verify_metric.py` audits specific joint velocities (`Joint_Drive_FL`, etc.) to ensure the drivetrain is operating within expected rad/s ranges.

---

*Testing analysis: 2025-05-15*
