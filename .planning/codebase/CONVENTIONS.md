# Coding Conventions

**Analysis Date:** 2024-05-21

## Naming Patterns

**Files:**
- Snake case for all scripts and modules: `verify_spawn.py`, `arcpro_env_cfg.py`, `track_manager.py`.
- Lowercase for directories: `arcproLab/`, `mdp/`, `scripts/`.
- UPPERCASE for shell scripts: `run_gui_verify.sh`, `train.sh`.

**Functions:**
- Snake case: `get_track_manager()`, `compute_errors()`, `reset_robot_to_lane()`.

**Variables:**
- Snake case: `env_cfg`, `robot_pos`, `lat_err`.

**Types/Classes:**
- PascalCase for configurations and classes: `ARCProEnvCfg`, `TrackManager`, `PolicyWrapper`.
- `@configclass` decorator is mandatory for Isaac Lab configuration classes.

## Code Style

**Formatting:**
- `flake8` for linting.
- Follow PEP 8 guidelines.

**Linting:**
- Configured in `requirements.txt` as a dev dependency.

## Import Organization

**Order:**
1. Standard library imports (`os`, `sys`, `argparse`).
2. Third-party library imports (`torch`, `numpy`, `matplotlib`).
3. Isaac Lab imports (`isaaclab.*`).
4. Local project imports (`arcpro_env_cfg`, `mdp.*`).

**Path Aliases:**
- `sys.path.append` is used in scripts to ensure local modules are discoverable:
  ```python
  sys.path.append(os.path.dirname(os.path.abspath(__file__)))
  ```

## 1.0x Metric Scaling Conventions

**Global Scale:**
- All physics entities MUST use 1.0x metric scale (meters, kilograms, seconds).
- `MetersPerUnit` should be `1.0`.

**Track Scaling:**
- OSM-based USD assets (e.g., `no_graph_sim_final.usd`) use a scale factor of `0.0825` to convert internal units to standard ~3.5m lane widths.
- Track Z-offset: `-1.25m` to ground the road surface at global `Z=0`.

**Robot Scaling:**
- Robot USD assets (e.g., `F1Tenth_Metric.usd`) use a scale factor of `1.0`.
- Robot spawn height: `0.5m` (drop onto grounded track).

**Telemetry Scaling:**
- Waypoints in `track_centerline.npy` must be pre-scaled by `0.0825` to match the 1.0x metric simulation.

## Error Handling

**Patterns:**
- Try-except blocks for optional component initialization (e.g., `TrackManager`, `TelemetryWindow`).
- Log error messages to console with clear prefixes like `[Verify]`.

## Logging

**Framework:** `print()` for script output and telemetry.

**Patterns:**
- Use formatted strings for telemetry output: `f"Step {count:4d} | Pos: ({pos[0,0]:.2f}, ...)"`.
- Regular intervals for logging (e.g., `if count % 20 == 0`).

## Module Design

**Exports:**
- Explicit imports from `mdp` submodules in `arcpro_env_cfg.py`.

**Barrel Files:**
- `mdp/__init__.py` used to group submodules.

---

*Convention analysis: 2024-05-21*
