# Testing Patterns

**Analysis Date:** 2025-04-28

## Test Framework

**Runner:**
- `pytest` for unit testing logic.
- Bash scripts for simulation-based verification (`verify_sim.sh`).

**Run Commands:**
```bash
pytest tests/           # Run logic tests
./verify_sim.sh         # Headless simulation performance audit
python arcproLab/scripts/verify_spawn.py --num_envs 16 # Spawn and camera verification
```

## Test File Organization

**Location:**
- Logic tests are separate in `tests/`.
- Simulation verification scripts are in `arcproLab/scripts/`.

**Naming:**
- `test_*.py` for unit tests.
- `verify_*.py` for simulation sanity checks.

## Unit Testing Structure

**Logic Testing:**
- Focus on `TrackManager` and vectorized math in `mdp/`.
- Uses `pytest` to verify waypoint generation and distance calculations.

## Simulation Verification Patterns

**Phase 09 Stabilization Loop:**
- **Sanity Check**: Run `verify_spawn.py` to ensure the robot starts on the road and cameras are correctly positioned.
- **Performance Audit**: Run `verify_sim.sh` to monitor FPS and physics stability at 8.0x scale.
- **Regression Check**: Run `verify_policy.py` with a known good model (e.g., `road_following_model.pth`) to ensure environment changes haven't broken the telemetry logic.

## Mocking

**Framework:**
- Not extensively used; simulation verification relies on running the actual Isaac Sim instance.

## Coverage

**Requirements:**
- Logic tests for critical math in `TrackManager` are expected.
- Coverage metrics are not strictly enforced.

## Test Types

**Unit Tests:**
- Waypoint error calculations.
- Tensor shape verification in `observations.py`.

**Integration Tests:**
- Robot-track alignment during reset events (`verify_spawn.py`).
- Full RL training loop convergence checks (`train.sh`).

**E2E Tests:**
- `run_gui_verify.sh` provides a manual end-to-end check of the trained policy in the simulation.

## Common Patterns

**Async Testing:**
- Simulation steps are blocking; tests iterate through fixed numbers of steps and verify state.

**Error Testing:**
- Testing fallback behavior in `events.py` when the robot is spawned far from the track.

---

*Testing analysis: 2025-04-28*
