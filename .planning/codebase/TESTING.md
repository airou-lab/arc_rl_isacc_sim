# Testing Patterns

**Analysis Date:** 2026-05-11

## Test Framework

**Runner:**
- pytest
- Config: `arcproLab/policy_stack/pytest.ini`

**Assertion Library:**
- pytest assertions.

**Run Commands:**
```bash
pytest arcproLab/policy_stack/tests/    # Run all MARL and policy unit tests
python arcproLab/scripts/verify_policy.py # Full simulation verification
```

## Test File Organization

**Location:**
- `arcproLab/policy_stack/tests/`

**Naming:**
- `test_*.py`

## Test Structure

**Suite Organization:**
```python
# arcproLab/policy_stack/tests/test_intersection_flow.py
def test_simultaneous_arrival():
    # Setup two agents at an intersection
    # Assert FCFS ordering and go_signal values
    pass
```

## Mocking

**Framework:**
- Manual mocks or `unittest.mock` if needed.

**Patterns:**
- Mocking the `SchedulerTransport` to test `WorkerScheduler` facades.
- Mocking the environment for testing `SchedulerCore` without simulation overhead.

## Fixtures and Factories

**Test Data:**
- `arcproLab/policy_stack/config/intersection_topology.json`: Sample topology for graph tests.

## Coverage

**Requirements:**
- High coverage required for `SchedulerCore` arbitration logic.

## Test Types

**Unit Tests:**
- `test_worker_scheduler.py`: Verifies facade delegation.
- `test_registry.py`: Verifies intent lifecycle.

**Integration Tests:**
- `test_intersection_flow.py`: Verifies multi-agent conflict resolution.

**Functional Tests:**
- `arcproLab/scripts/verify_*.py`: End-to-end simulation tests for drive, metric, and reset logic.

## Common Patterns

**Async Testing:**
- Used for `IntersectionNodeServer` tests if network transports are involved.

**Error Testing:**
- Verifying stale intent expiration via `SchedulerCore.tick()`.

---

*Testing analysis: 2026-05-11*
