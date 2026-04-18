# Coding Conventions

**Analysis Date:** 2025-05-21

## Naming Patterns

**Files:**
- [snake_case.py]: All modules and scripts.
- [snake_case_1x.usda]: Assets specifically scaled to 1.0x metric units.

**Functions:**
- [snake_case]: `get_telemetry_vector()`, `white_line_contact()`, `compute_marker_distances()`.

**Variables:**
- [snake_case]: `dist_y`, `dist_w`, `marker_hit`, `local_pos`.
- Legacy names like `lat_err` and `head_err` may still appear in `env.extras` for compatibility but are deprecated.

**Types:**
- Use [CamelCase] for `@configclass` definitions: `ARCProEnvCfg`.

## Code Style

**Formatting:**
- Standard PEP8.
- Indentation: 4 spaces.

**Linting:**
- Syntax verification via GitHub Actions.

## Import Organization

**Order:**
1. Standard library (`os`, `sys`, `math`).
2. Third-party (`torch`, `numpy`).
3. Omniverse/Isaac (`omni`, `pxr`, `isaaclab`).
4. Internal modules (`mdp.*`, `arcpro_env_cfg`).

**Path Aliases:**
- Use `sys.path.append(os.path.dirname(os.path.abspath(__file__)))` at the top of configuration files to allow local imports.

## Error Handling

**Patterns:**
- **Zero-Masking**: Always mask NaNs in observation tensors (`obs[nan_mask] = 0.0`).
- **Vision-Only Masking**: Explicitly zero out telemetry indices 8 & 9 in `observations.py` to force vision reliance.
- **Grace Periods**: Use `env.episode_length_buf > 20` to allow physics to settle before applying strict terminations.

## Logging

**Framework:**
- `print()` for real-time termination reasons (e.g., "[TERMINATION] Yellow Boundary Hit!").
- `env.extras` for storing raw proximity data used by rewards and terminations.

## Comments

**When to Comment:**
- Document termination thresholds (e.g., "Reset if robot center is within 0.1m of any marker").
- Explain USD stage traversal logic in `TrackManager`.

**JSDoc/TSDoc:**
- Use standard docstrings for all MDP functions.

## Function Design

**Size:**
- Keep MDP functions (rewards, terminations) focused and under 50 lines.

**Parameters:**
- Primary parameter is always `env: ManagerBasedRLEnv`.

**Return Values:**
- MDP functions MUST return `torch.Tensor` of shape `(num_envs,)`.

## Module Design

**Exports:**
- Group functional logic in `arcproLab/mdp/`.

**Config Classes:**
- Inheritance-based configuration using `replace()` for specific variations.

---

*Convention analysis: 2025-05-21*
