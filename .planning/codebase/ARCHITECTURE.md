# Architecture

**Analysis Date:** 2025-05-21

## Pattern Overview

**Overall:** Modular RL Environment based on NVIDIA Isaac Lab's Manager-Based RL pattern, utilizing a 1.0x true metric physics scale.

**Key Characteristics:**
- **Metric Physics Parity:** Robot is defined at 1.0x scale (meters) with realistic mass (20kg) and inertia. The environment USD is scaled to 1.0x (from 8x source) to maintain metric consistency.
- **Vision-First Navigation:** Telemetry observations are masked at the policy level to force reliance on camera input, while raw telemetry is used for reward and termination logic.
- **Direct Proximity Boundary Detection:** Replaces problematic centerline-based lateral error math with robust, marker-based proximity queries. The system identifies yellow and white road markers directly from the USD stage.

## Layers

**Configuration Layer:**
- Purpose: Defines the environment, robot, and MDP (Markov Decision Process) structure.
- Location: `arcproLab/arcpro_env_cfg.py`, `arcproLab/arcpro_robot_cfg.py`.
- Contains: `ARCProEnvCfg`, `ARCProSceneCfg`, `RewardCfg`, `TerminationCfg`.
- Depends on: `isaaclab`, `mdp` logic.
- Used by: Scripts in `arcproLab/scripts/`.

**MDP (Logic) Layer:**
- Purpose: Implements observations, rewards, and terminations.
- Location: `arcproLab/mdp/`.
- Contains: `observations.py`, `rewards.py`, `terminations.py`, `track_manager.py`.
- Depends on: `torch`, `isaaclab`, `omni.usd`.
- Used by: `arcproLab/arcpro_env_cfg.py`.

**Asset Layer:**
- Purpose: Provides the physical and visual representation of the world.
- Location: `openStreetUSD/`, `arcproLab/assets/robot/`.
- Contains: `no_graph_sim_clean_1x.usda`, `F1Tenth_Metric.usd`.
- Depends on: Universal Scene Description (USD).

## Data Flow

**Observation Pipeline:**
1. **Raw State**: Positions, velocities, and quaternions extracted from the simulation.
2. **Telemetry Computation**: `mdp.observations.get_telemetry_vector` calculates 12 features (odom, speed, yaw rate, errors, distance).
3. **Proximity Calculation**: `TrackManager` computes distances to the nearest yellow and white markers via `compute_marker_distances` using `torch.cdist`.
4. **Error Masking**: Indices 8 (LatErr) and 9 (HeadErr) are zeroed out before reaching the policy to force vision reliance.
5. **Extras**: Raw proximity and speed data are stored in `env.extras` for use by the Reward and Termination managers.

**Action Pipeline:**
1. **Policy Output**: Steer (-1 to 1) and Throttle (0 to 1).
2. **Scaling**: Actions are scaled (e.g., Throttle x 60.0 rad/s) and applied to joint position/velocity targets via `ActionCfg`.

## Key Abstractions

**TrackManager:**
- Purpose: Singleton providing distance-to-boundary queries.
- Pattern: Automatically collects marker points ("yellow" or "white" materials/paths) from the USD stage on initialization.
- Logic: `arcproLab/mdp/track_manager.py`.

**12-Element Telemetry Protocol:**
- Purpose: Feature vector ensuring continuity across training phases.
- Definition: `arcproLab/mdp/observations.py`.

## Entry Points

**Training Entry:**
- Location: `arcproLab/scripts/train_policy.py`.
- Action: Sets up `ManagerBasedRLEnv` and runs SB3 PPO.

**Verification Entry:**
- Location: `arcproLab/scripts/verify_live.py`.
- Action: Visual inspection with `TelemetryWindow` UI (`mdp/visual_analytics.py`).

## Error Handling

**Strategy:** Strict environment resets using direct proximity thresholds to prevent learning from out-of-bounds states.

**Patterns:**
- **NaN Zeroing**: `observations.py` zeros out NaNs to prevent policy weights from exploding.
- **Direct Proximity Termination**: `white_line_contact` resets the environment if the robot center is within 0.1m of any road marker.
- **FOV Protection**: `fov_visibility_termination` resets if the velocity vector diverges from the camera's visual cone.

## Cross-Cutting Concerns

**Logging:** Training metrics go to `logs/`, runtime telemetry displayed via `omni.ui`.
**Coordinate Frames:** Standardizes on `root_pos_w - env_origins` for environment-local calculations.

---

*Architecture analysis: 2025-05-21*
