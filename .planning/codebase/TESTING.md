# Testing Patterns

**Analysis Date:** 2024-11-20

## Test Framework

**Runner:**
- pytest
- Config: `arcproLab/policy_stack/pytest.ini`

**Assertion Library:**
- Standard Python `assert`.

**Run Commands:**
```bash
pytest arcproLab/policy_stack/tests/    # Run policy stack unit tests
python arcproLab/scripts/verify_policy.py --checkpoint models/model.pth # Run simulation verification
```

## Test File Organization

**Location:**
- Unit tests: `arcproLab/policy_stack/tests/`
- Functional/Simulation tests: `arcproLab/scripts/` (prefixed with `verify_` or `test_`)

**Naming:**
- Unit tests: `test_*.py`
- Simulation tests: `verify_*.py` or `test_*.py`

## Test Structure

**Suite Organization:**
```python
# arcproLab/policy_stack/tests/test_registry.py
def test_registry_registration():
    # Test logic
    pass

def test_registry_make():
    # Test logic
    pass
```

**Patterns:**
- Functional scripts: Many scripts in `arcproLab/scripts/` follow a "Launch Simulation -> Load Model -> Run N steps -> Print Results" pattern.

## Mocking

**Framework:** None explicitly used; simulation environments are typically used directly or via Direct Envs.

**Patterns:**
- Isaac Lab `DirectRLEnv` used to simplify testing environments without the full manager overhead.

## Fixtures and Factories

**Test Data:**
- Cached track boundaries in `.npz` files.
- Sample models in `arcproLab/models/`.

**Location:**
- `arcproLab/mdp/*.npz`
- `arcproLab/models/*.pth`

## Coverage

**Requirements:** None enforced.

## Test Types

**Unit Tests:**
- Test individual RL components (registry, schedulers) in `arcproLab/policy_stack/tests/`.

**Integration Tests:**
- Testing the intersection flow and node server in `arcproLab/policy_stack/tests/`.

**Simulation Verification:**
- Extensive collection of scripts to verify specific behaviors:
  - `verify_metric.py`: Scale/unit verification.
  - `verify_drive.py`: Actuator verification.
  - `verify_spawn.py`: Reset/Spawn logic verification.

## Common Patterns

**Async Testing:**
- Not prevalent; simulation is typically synchronous per-step.

**Error Testing:**
- Check for initialization failures (e.g., `TrackManager` sync attempts).

---

*Testing analysis: 2024-11-20*
