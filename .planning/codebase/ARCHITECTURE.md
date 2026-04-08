# Architecture

**Analysis Date:** 2025-04-28

## Pattern Overview

**Overall:** Manager-Based RL with **Phase-Driven Stabilization** workflow. This architecture focuses on iterative verification of the hybrid 8.0x metric scale environment before large-scale training.

**Key Characteristics:**
- **Manager-Based RL**: Strict decoupling of Scene, Observation, Action, Reward, and Termination logic using Isaac Lab's `ManagerBasedRLEnv`.
- **Hybrid Scale (8.0x)**: The robot and world are scaled to 8.0x to avoid precision issues in Isaac Sim physics, while observation and reward logic normalize to 1.0x metric units.
- **Phase 09 Stabilization Workflow**: A task-oriented approach integrating automated verification (`verify_spawn.py`, `verify_metric.py`) with manual "sanity" checks (visual/camera offset, torque audit).
- **True-Physics Simulation**: High-fidelity vehicle dynamics with 200Hz control loop and TGS solver.

## Layers

**Configuration Layer:**
- Purpose: Defines simulation parameters, assets, and robot properties.
- Location: `arcproLab/arcpro_env_cfg.py`, `arcproLab/arcpro_robot_cfg.py`.
- Features: Configurable camera status (`enable_cameras`), 20kg chassis mass, and 4WD actuator settings.

**MDP Layer (Markov Decision Process):**
- Purpose: Bridges simulation state to RL policy.
- Location: `arcproLab/mdp/`.
- Key Logic:
  - `observations.py`: Implements the 12-float telemetry protocol with 0.125 scale normalization.
  - `events.py`: Implements `reset_robot_to_lane` with robust raycast snapping to the road surface.
  - `track_manager.py`: Centralized waypoint management and error calculation.

**Verification & Audit Layer:**
- Purpose: New layer introduced for Phase 09 stabilization.
- Location: `arcproLab/scripts/`.
- Tools: `verify_spawn.py` (spawning/alignment), `audit_assets.py` (USD integrity), `verify_metric.py` (physical performance).

**Policy Layer:**
- Purpose: SB3-based PPO training and evaluation.
- Location: `arcproLab/scripts/train_policy.py`, `arcproLab/scripts/verify_policy.py`.
- Integration: Uses `isaaclab_rl.sb3.Sb3VecEnvWrapper` for environment adaptation.

## Data Flow

**Phase 09 Stabilization Loop:**

1. **Configuration**: Set flags (e.g., `enable_cameras`) in `arcpro_env_cfg.py`.
2. **Local Verification**: Run `verify_spawn.py` with GUI to check camera offsets and robot initialization.
3. **Physical Audit**: Execute `verify_sim.sh` to perform a "Torque Audit" (acceleration and hill-climbing checks).
4. **Integration**: Commit stabilized configs to the dev branch.
5. **Training**: Launch `train.sh` for converged model generation.

**Simulation Step:**

1. **Event Reset**: `mdp/events.py` snaps robot to road at Z=0.05m (target) or 1.0m (current) with orientation.
2. **Observation**: Raw simulation values → Normalized 1.0x metric vector.
3. **Policy**: Model output → Wheel steering and drive efforts.
4. **Physics**: PhysX 5.1 step at 200Hz.
5. **Reward/Termination**: Calculation based on lateral error and track progress.

## Key Abstractions

**12-Float Telemetry Protocol:**
- Standardized observation vector for the navigation policy.
- Contains: Velocity, heading error, lateral error, and look-ahead waypoints.

**TrackManager (Singleton):**
- Vectorized waypoint management.
- Handles distance-to-curve calculations and point-in-lane verification.

## Entry Points

**Verification:**
- `arcproLab/scripts/verify_spawn.py`: Primary tool for checking robot/track alignment and camera setup.

**Training:**
- `arcproLab/scripts/train_policy.py`: Main entry for SB3 PPO training.

**Evaluation:**
- `arcproLab/scripts/verify_policy.py`: Used for checking trained model performance in the environment.

## Error Handling

**Strategy:** Fail-fast on configuration/asset mismatches; robust fallback in simulation events.

**Patterns:**
- `reset_robot_to_lane` includes a 50-retry raycast loop with broad road mesh detection.
- Fallback to Z=10.0 and hardcoded waypoints if road snapping fails.

## Cross-Cutting Concerns

**Logging:** Standard TensorBoard logs for RL training; stdout for verification tools.
**Scale Normalization:** Centralized in `mdp/observations.py` to prevent "8x leak" into the policy.
**Torque/Power:** Actuators calibrated for 20kg mass at 8x scale (stiffness/damping tuning).

---

*Architecture analysis: 2025-04-28*
