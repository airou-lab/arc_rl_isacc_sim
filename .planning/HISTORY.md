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

### 1. 1400kg Balanced Mass Distribution
- **Issue**: Initial increase to 1500kg was applied uniformly (200kg per link), putting 1200kg in the wheels/knuckles and only 200kg in the chassis. This caused massive steering drift (16° in 10 steps) and "jumping" physics.
- **Solution**: 
    - Implemented a custom spawner (`arcproLab/mdp/spawner.py`) using `schemas.define_mass_properties` to apply link-specific mass.
    - Balanced distribution: **1200kg Chassis**, **75kg Wheels**, **10kg Knuckles**.
    - Elevated spawn height to **1.0m** (`mdp/events.py`) to prevent 3.6m wide wheels from clipping into the ground.
    - Result: Steering bias reduced to **<2° over 100 steps**.

### 2. Action Space Refactor (2D Control)
- **Issue**: Standard `JointAction` terms expanded regex expressions into 6 individual dimensions (2 steer, 4 drive). The agent struggled to coordinate 4 wheels synchronously, leading to excessive spin and slow learning.
- **Solution**: 
    - Created custom `GroupedJointAction` classes in `arcproLab/mdp/actions.py`.
    - Collapsed the action space into **2 dimensions**: 1 Steering (mapped to L/R) and 1 Throttle (mapped to all 4 wheels).
    - Result: synchronous AWD driving and faster policy convergence.

### 4. Normalization Sync & Learning Surge
- **Issue**: Visual verification script (`verify_live.py`) showed robot stuck with 0.0 speed despite high throttle commands. GUI behavior did not match the policy's intended actions.
- **Solution**: 
    - Discovered that the policy was trained on **Normalized Observations**, but the viewer was feeding it **Raw World Coordinates**.
    - Updated `train_policy.py` with `SaveVecNormalizeCallback` to save `vec_normalize.pkl` every 10,000 steps.
    - Result: Confirmed training convergence with `ep_rew_mean` rising from -3.09 to +24.9 and `explained_variance` reaching 0.901.


### 2026-04-18: Shift to Direct Proximity Termination
- **Problem**: Centerline-based lateral error logic was unstable due to procedural waypoint generation failures on the 1.0x map, causing "phantom" resets in the middle of the road.
- **Solution**: Implemented robust direct-distance termination.
  - TrackManager now stores raw Yellow/White marker points as GPU tensors.
  - Terminations are triggered if the robot center is < 0.1m from any marker point.
  - This removes all dependency on "centerline" or "lane" ordering math.
- **Result**: Verified in GUI that the robot drives through the lane and only resets upon physical boundary contact.
