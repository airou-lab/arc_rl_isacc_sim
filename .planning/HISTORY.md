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

## Phase 09: Training Loop Stabilization

### 1. 1500kg Physics Stabilization
- **Issue**: The 8.0x scale robot (3.6m wide) was behaving like a "balloon" with a 20kg mass, jittering and flipping during resets.
- **Solution**: 
    - Increased mass to 1500kg in `arcpro_robot_cfg.py`.
    - Boosted actuator stiffness to 50,000 and damping to 1,000 to handle the high inertia.
    - Result: Stable, heavy chassis behavior with realistic tire friction.

### 2. AWD Joint Mapping & Inversion
- **Issue**: Actuator commands were only affecting the rear wheels, and wheel directions were inconsistent (negative rad/s caused spin-outs).
- **Solution**: 
    - Updated `ActionCfg` to use regex `Joint_Drive_.*` to capture all 4 motors.
    - Verified joint indices: 0,1 (Steer), 2,3 (Rear), 4,5 (Front).
    - Fixed direction signs: +40.0 rad/s now drives all wheels forward.

### 3. Reset Logic & Multi-Env Bug
- **Issue**: Hardcoded world coordinates in `events.py` caused robots in `Env 1` to spawn in empty space when training with `num_envs > 1`.
- **Solution**: 
    - Temporarily reduced training to 1 environment to ensure math alignment.
    - Implemented a 0.6m lateral drift limit from the mathematical centerline (X=-130.03).
    - Added a 20-step "settling" grace period to prevent reset-loops during physics initialization.

