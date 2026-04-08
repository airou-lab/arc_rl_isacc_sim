# Coding Conventions

**Analysis Date:** 2025-04-28

## Naming Patterns

**Files:**
- Snake case for all scripts and modules: `arcpro_env_cfg.py`, `train_policy.py`.
- Lowercase for asset directories: `assets/`, `openStreetUSD/`.

**Functions:**
- Snake case: `reset_robot_to_lane()`.

**Classes:**
- CamelCase for configuration classes: `ARCProEnvCfg`, `ARCProSceneCfg`.

**Types:**
- Use `torch.Tensor` for vectorized RL state; use standard Python types for configuration flags.

## Code Style

**Formatting:**
- Flake8 for linting (implied by `requirements.txt`).
- Standard Python style following Isaac Lab developer guidelines (PEP8).

**Linting:**
- `flake8` for syntax and style checking.

## Import Organization

**Order:**
1. Standard library imports.
2. Third-party library imports (e.g., `torch`, `omni`).
3. Local application imports (e.g., `arcpro_env_cfg`).

**Path Aliases:**
- `sys.path.append(os.path.join(os.path.dirname(__file__), ".."))` is used in scripts to access `arcproLab` modules.

## Error Handling

**Patterns:**
- Extensive retry loops for stochastic simulation operations (e.g., `reset_robot_to_lane`).
- Fallback positions and warning logs when critical simulation geometry cannot be detected.

## Logging

**Framework:**
- `print()` for local script output.
- `TensorBoard` for training progress via Stable Baselines3.

## Workflow Conventions

**Stabilization Tasks:**
- Use `.planning/todos/` for tracking discrete stabilization tasks.
- Each todo should have a clear Goal and a list of actionable Tasks.
- Phase 09 specifically uses `verify_spawn.py` as a mandatory sanity check before code promotion.

## Module Design

**Exports:**
- Use `__init__.py` to expose key logic in `arcproLab/mdp/`.

**Config Classes:**
- Use `@configclass` decorator from `isaaclab.utils.configclass` for all environment and robot configurations.

---

*Convention analysis: 2025-04-28*
