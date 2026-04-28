# Coding Conventions

**Analysis Date:** 2024-04-23

## Naming Patterns

**Files:**
- Snake Case: `arcpro_env_cfg.py`, `track_manager.py`.

**Functions:**
- Snake Case: `speed_reward(env, ...)`, `get_telemetry_vector(...)`.

**Variables:**
- Snake Case: `lat_err`, `head_err`, `turn_token`.

**Types:**
- PascalCase for Config classes: `ARCProEnvCfg`, `ARCProSceneCfg`.
- PascalCase for Managers: `TrackManager`, `RoadGraph`.

## Code Style

**Formatting:**
- Follows Isaac Lab standards (largely PEP8).
- Copyright headers included in most core files.

**Linting:**
- Configured in `.github/workflows/ci.yml`.
- Standard Python linting (flake8/black/isort usually assumed in Isaac projects).

## Import Organization

**Order:**
1. Standard Library (`os`, `sys`, `math`).
2. Major Frameworks (`torch`, `numpy`).
3. Isaac Lab / Omniverse (`isaaclab.*`, `omni.*`).
4. Project Local Modules (`arcpro_robot_cfg`, `mdp.*`).

**Path Aliases:**
- `mdp` is often imported as a local package within `arcproLab`.
- `sys.path.append` is used in several scripts to ensure cross-package visibility (e.g., `arcproLab/arcpro_env_cfg.py`).

## Error Handling

**Patterns:**
- **NaN Safeguards:** Mandatory in reward and observation functions to prevent policy divergence.
- **Existence Checks:** `if not root_prim.IsValid(): ...` when interacting with the USD stage.
- **Graceful Failures:** Scripts like `train_policy.py` use `try-except` blocks for non-critical components like action history.

## Logging

**Framework:** `console` and `Tensorboard`.

**Patterns:**
- **Debug Flags:** `TrackManager` uses a `--debug` CLI flag to toggle expensive visualization markers.
- **Periodic Logging:** Telemetry debug prints in `observations.py` are throttled (e.g., `episode_length_buf[0] % 10 == 0`).

## Comments

**When to Comment:**
- Complexity: Explain geometric calculations (e.g., yaw from quaternion).
- Calibration: Document scale changes (e.g., 8x to 1x camera offsets).

**JSDoc/TSDoc:**
- Python Docstrings (`"""Docstring"""`) used for classes and main functions.

## Function Design

**Size:** Preference for small, focused functions in `mdp/`.

**Parameters:** Manager functions typically take `env: ManagerBasedRLEnv` as the first argument.

**Return Values:** MDP manager terms MUST return `torch.Tensor` of shape `(num_envs, ...)`.

## Module Design

**Exports:** Standard Python module exports.

**Barrel Files:** `arcproLab/mdp/__init__.py` used to aggregate MDP terms.

---

*Convention analysis: 2024-04-23*
