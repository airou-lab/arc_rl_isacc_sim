# Coding Conventions

**Analysis Date:** 2025-05-20

## Naming Patterns

**Files:**
- [snake_case.py]: All modules and scripts.
- [snake_case_1x.usda]: Assets specifically scaled to 1.0x metric units.

**Functions:**
- [snake_case]: `get_telemetry_vector()`, `white_line_contact()`.

**Variables:**
- [snake_case]: `lat_err`, `head_err`, `local_pos`.

**Types:**
- Use [CamelCase] for `@configclass` definitions: `ARCProEnvCfg`.

## Code Style

**Formatting:**
- Standard PEP8. Use black or equivalent if available.
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
- **Grace Periods**: Use `env.episode_length_buf > 20` to allow physics to settle before applying strict terminations (e.g., FOV checks).

## Logging

**Framework:**
- `print()` for real-time termination reasons and script debug info.
- `env.extras` for storing telemetry used by rewards and terminations.

## Comments

**When to Comment:**
- Explain magic numbers (e.g., mass overrides, PID gains, sensor offsets).
- Document termination thresholds and reward weights.

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
- Inheritance-based configuration using `replace()` for specific variations (e.g., `ARCPRO_ROBOT_CFG.replace(...)`).

---

*Convention analysis: 2025-05-20*
