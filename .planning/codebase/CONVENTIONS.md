# Coding Conventions

**Analysis Date:** 2026-05-11

## Naming Patterns

**Files:**
- snake_case: `road_manager.py`, `scheduler_core.py`.

**Functions:**
- snake_case: `register_intent()`, `update_road_manager()`.

**Variables:**
- snake_case: `turn_tokens`, `go_signals`.

**Types:**
- CamelCase for classes: `RoadManager`, `SchedulerCore`.
- Use typing hints for all public methods.

## Code Style

**Formatting:**
- Black/PEP8 compliant.
- BSD-3-Clause headers required for core `arcproLab` files.

**Linting:**
- Configured via `pytest.ini`.

## Import Organization

**Order:**
1. Future imports (`from __future__ import annotations`).
2. Standard library.
3. Third-party (torch, numpy).
4. Local modules.

**Path Aliases:**
- Use absolute imports within `arcproLab` where possible.
- Avoid circular dependencies in `policy_stack` by using local imports in methods if necessary (see `WorkerScheduler`).

## Error Handling

**Patterns:**
- Graceful degradation if USD prims are missing (see `RoadManager._initialize_gates`).
- Logging warnings for dead code or configuration mismatches.

## Logging

**Framework:**
- `logging` module for system-level messages.
- `print` with prefixes for simulation-level messages (e.g., `[RoadManager] ...`).
- Tensorboard for RL metrics.

## Comments

**When to Comment:**
- Complexity: Explain coordinate transformations (Vehicle vs World frame).
- Policy: Explain inductive biases (e.g., Kinematic Anchors).

**JSDoc/TSDoc:**
- Standard Python Docstrings.

## Function Design

**Size:**
- Prefer small, pure-compute functions for arbitration (see `scheduler_core.py`).
- Keep environment update loops centralized.

## Module Design

**Exports:**
- Explicit `__all__` in facade modules (`worker_scheduler.py`).

**Barrel Files:**
- `__init__.py` should expose core classes for cleaner imports.

---

*Convention analysis: 2026-05-11*
