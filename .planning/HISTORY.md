# Project History: Phase 07 Modernization

## Issues Encountered & Resolved

### 1. Gymnasium 1.0+ Migration
- **Issue**: Standard `import gym` triggered "Gym is unmaintained" warnings. `import gymnasium as gym` still triggered warnings in `AppLauncher`.
- **Solution**: 
    - Moved the `warnings.filterwarnings` to the absolute top of `train_policy.py`, before any other imports.
    - Updated `arcproLab/__init__.py` to use `gymnasium.register`.
    - Standardized all `import gymnasium as gym` across the codebase.

### 2. USD Reference Pollution
- **Issue**: `no_graph_sim.usd` had broken links to `Flattened_Master` and a missing `f1tenth_trainer/assets/` folder, causing dozens of "Asset not found" errors.
- **Solution**: 
    - Created a Python repair script (`trash/tools/repair_usd_references.py`) to surgically remove the dead `F1Tenth` prim.
    - Redirected 24+ broken signpost references to a valid local placeholder (`F1Tenth_Metric.usd`).
    - Successfully silenced all USD console warnings.

### 3. Simulation Configuration (`SimulationCfg`)
- **Issue**: Attempting to use `enable_stabilization` or `gpu_max_rigid_contact_count` directly in `SimulationCfg` (Isaac Lab v2.x) caused `TypeError`.
- **Solution**: 
    - Reverted to the core baseline in `arcpro_env_cfg.py`.
    - Corrected the `PhysxCfg` nesting to only include supported parameters: `enable_ccd` and `enable_external_forces_every_iteration`.

### 4. AWD Torque vs. Mass
- **Issue**: Robot (20kg) was struggling to move reliably against 5.0 damping.
- **Solution**: 
    - Enabled `enable_external_forces_every_iteration=True` in `PhysxCfg`.
    - Verified all 4 drive joints (`Joint_Drive_.*`) are correctly mapped in `ActionCfg`.

## Remaining Blockers (Phase 08)
- **Road Detection**: The raycast in `events.py` is failing to find the `drivable_surfaces` meshes due to a vertical range issue or offset.
