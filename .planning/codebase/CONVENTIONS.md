# Coding Conventions

**Analysis Date:** 2025-05-15

## Naming Patterns

**Files:**
- Snake case for all scripts and modules: `arcpro_env_cfg.py`, `train_policy.py`.
- Lowercase for asset directories: `assets/`, `openStreetUSD/`.
- `_1x` suffix for 1x-metric scaled assets: `no_graph_sim_clean_1x.usda`.

**Functions:**
- Snake case: `get_telemetry_vector()`, `fov_visibility_termination()`.

**Classes:**
- CamelCase for configuration classes: `ARCProEnvCfg`, `ARCProSceneCfg`.

**Types:**
- Use `torch.Tensor` for vectorized RL state; use standard Python types for configuration flags.

## Code Style

**Formatting:**
- Standard Python style following Isaac Lab developer guidelines (PEP8).

**Linting:**
- `flake8` for syntax and style checking.

## Import Organization

**Order:**
1. Standard library imports.
2. Third-party library imports (`torch`, `omni`, `isaaclab`).
3. Local application imports (`mdp.observations`, `arcpro_env_cfg`).

**Path Aliases:**
- Use `sys.path.append(os.path.dirname(os.path.abspath(__file__)))` in config files.
- Use root-relative imports where possible in scripts.

## Error Handling

**Patterns:**
- **NaN Protection**: Explicitly zero out NaNs in observation vectors.
- **Physics Snapping**: Use raycasting in `mdp.events` to snap the robot to road height during reset.
- **Terminal Debugging**: Print clear termination reasons (e.g., "[TERMINATION] Driving Blind!") to stdout for easier debugging.

## Logging

**Framework:**
- `print()` for local script output and real-time termination debugging.
- `TensorBoard` for training progress via Stable Baselines3.

## Workflow Conventions

**Metric Verification:**
- Always run `verify_metric.py` after changes to physics or assets to ensure 1x scale integrity.
- Use `run_gui_verify.sh` for manual visual inspection of policy behavior and camera FOV.

## Module Design

**Exports:**
- Use `__init__.py` to expose key logic in `arcproLab/mdp/`.

**Config Classes:**
- Use `@configclass` decorator from `isaaclab.utils.configclass` for all environment and robot configurations.
- Centralize all magic numbers (scales, offsets, thresholds) in `@configclass` objects in `arcpro_env_cfg.py`.

---

*Convention analysis: 2025-05-15*
