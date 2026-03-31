# Coding Conventions

**Analysis Date:** 2024-10-24

## Naming Patterns

**Files:**
- snake_case: `arcpro_env_cfg.py`, `track_manager.py`.

**Functions:**
- snake_case: `compute_reward()`, `get_observations()`.

**Variables:**
- snake_case: `joint_pos`, `throttle_limit`.

**Types / Classes:**
- PascalCase: `ArcProRobotCfg`, `HierarchicalPolicy`, `TrackManager`.

**Constants:**
- UPPER_CASE: `ARCPRO_ROBOT_CFG`, `WAYPOINT_NORM_SCALE`.

## Code Style

**Formatting:**
- flake8 (implied by `requirements.txt`).
- Standard Python (PEP 8) with emphasis on `@configclass` usage for Isaac Sim.

**Linting:**
- flake8 for syntax and style checking.

## Import Organization

**Order:**
1. Standard library (`os`, `sys`, `math`).
2. Major third-party libraries (`torch`, `numpy`).
3. Isaac Sim / IsaacLab imports (`isaaclab.sim`, `isaaclab.envs`).
4. Local project imports (`arcpro_robot_cfg`, `mdp.observations`).

**Path Aliases:**
- None detected; uses relative imports and `sys.path.append(os.path.dirname(os.path.abspath(__file__)))` in `arcpro_env_cfg.py`.

## Error Handling

**Patterns:**
- Use of `warnings.warn` for sim-related warnings that shouldn't crash training.
- `try-except` blocks around complex SB3-contrib logic (e.g., RNN state handling).

## Logging

**Framework:** TensorBoard (integrated with SB3) and standard Python `logging` / `print`.

**Patterns:**
- Log to TensorBoard during training for metrics like `reward`, `loss`, `episode_length`.
- `print` or `logging.info` for environment setup details.

## Comments

**When to Comment:**
- Complexity: Commenting complex math in `track_manager.py`.
- Reasoning: Commenting design decisions in `hierarchical_policy.py` (e.g., why a certain normalization scale was used).

**JSDoc/TSDoc:**
- Docstrings are standard for classes and functions, often following the IsaacLab style (brief summary followed by parameter descriptions).

## Function Design

**Size:** Preference for modular functions in `mdp/` submodules.

**Parameters:** Use of keyword arguments and IsaacLab `SceneEntityCfg` for identifying assets.

**Return Values:** Usually return tensors (for RL) or boolean/scalars (for rewards/terminations).

## Module Design

**Exports:** Explicitly defining `ARCPRO_ROBOT_CFG` in config files.

**Barrel Files:** `__init__.py` used to expose common names in `mdp/`.

---

*Convention analysis: 2024-10-24*
