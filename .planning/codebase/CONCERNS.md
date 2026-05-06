# Codebase Concerns

**Analysis Date:** 2024-04-14

## Tech Debt

**USD-Based Marker Categorization:**
- Issue: `TrackManager` and `RoadGraph` rely on traversing the USD stage and matching prim names/materials (e.g., "yellow", "white", "laneGate") to build logical boundaries and navigation intents.
- Files: `arcproLab/mdp/track_manager.py`, `arcproLab/mdp/road_graph.py`
- Impact: Brittle. If USD asset names change or material paths vary, the simulation logic fails to detect track boundaries or navigation gates.
- Fix approach: Implement a more robust tagging system (e.g., USD Attributes/Semantics) or move boundary definitions to a dedicated configuration file/map format.

**Hardcoded Simulation Offsets:**
- Issue: Numerous hardcoded values for lane offsets, marker distances, and geometry.
- Files: `arcproLab/mdp/track_manager.py` (line 173), `arcproLab/mdp/rewards.py`, `arcproLab/mdp/terminations.py`
- Impact: Difficult to generalize to different track layouts or robot dimensions without manual code changes.
- Fix approach: Centralize geometry-dependent constants in `ArcProEnvCfg` or load them from the track USD metadata.

**Stateful Observation Function:**
- Issue: `get_telemetry_vector` performs updates to `RoadGraph` and `env.extras["distance"]`, meaning the observation gathering has side effects.
- Files: `arcproLab/mdp/observations.py`
- Impact: Violates the functional expectation of observation gathering. Could lead to double-counting if observations are requested multiple times per step (e.g., for logging or visualization).
- Fix approach: Move state updates (distance accumulation, road graph updates) to an `ActionManager` or a dedicated `StepHook`.

## Known Bugs

**NaN Propagation in Rewards/Observations:**
- Issue: Explicit `torch.isnan` checks and zero-filling are present in multiple MDP files.
- Files: `arcproLab/mdp/rewards.py`, `arcproLab/mdp/observations.py`
- Symptoms: Occasional zeroing of rewards or observations when physics or math (atan2/cdist) produces NaNs.
- Trigger: Likely extreme physics states or collisions at 1.0x metric scale.
- Workaround: Zero-filling keeps training running but hides underlying physics instability.

## Security Considerations

**Unrestricted Shell Execution in `TrackManager`:**
- Risk: `TrackManager` checks `sys.argv` for `--debug` to enable visuals.
- Files: `arcproLab/mdp/track_manager.py`
- Current mitigation: None.
- Recommendations: Use a proper configuration system rather than raw `sys.argv` to control simulation features.

## Performance Bottlenecks

**Iterative USD Stage Traversal:**
- Problem: Building boundary tensors from USD meshes is extremely slow on startup.
- Files: `arcproLab/mdp/track_manager.py` (`collect_raw_marker_points`)
- Cause: Traverses every prim in the stage to find markings.
- Improvement path: Optimize the `save_cache`/`load_cache` mechanism and use `omni.usd` query APIs instead of a full `Usd.PrimRange` traversal.

**Distance Calculations (cdist):**
- Problem: `compute_marker_distances` performs `torch.cdist` against dense marker tensors every step.
- Files: `arcproLab/mdp/track_manager.py`, `arcproLab/mdp/terminations.py`
- Cause: Dense point clouds for boundaries (Yellow/White/Gate) lead to large distance matrices.
- Improvement path: Use a spatial hash or KD-Tree (ideally GPU-accelerated) for proximity checks.

## Fragile Areas

**Low-Mass Physics Stability:**
- Files: `arcproLab/arcpro_robot_cfg.py`, `arcproLab/mdp/spawner.py`
- Why fragile: Reducing robot mass to 5kg at 1.0x scale (metric) pushes the PhysX solver limits. Small mass ratios between joints and high actuator gains lead to numerical "jitter."
- Safe modification: Follow scaling laws for motor parameters and maintain high solver iterations (16-32) and `armature`.
- Test coverage: Gaps in automated physics validation scripts.

**Gate Permeability Logic:**
- Files: `arcproLab/mdp/terminations.py` (`white_line_contact`)
- Why fragile: Logic determines if a "Gate" (Stop Line) is a wall or permeable based on robot heading and speed. This heuristic might fail during complex maneuvers or at high slip angles.
- Safe modification: Tie gate permeability to `RoadGraph` intent (e.g., if intent is 'Straight', the 'Straight' gate is permeable).

## Scaling Limits

**Metric Scale Precision:**
- Current capacity: 1.0x scale (meters).
- Limit: Numerical precision issues in PhysX when handling small collisions (0.01m) at high speeds.
- Scaling path: Maintain high frequency `dt=0.002` (500Hz) and TGS solver.

## Test Coverage Gaps

**MDP Core Logic:**
- What's not tested: Reward functions, observation vector correctness, termination triggers.
- Files: `arcproLab/mdp/rewards.py`, `arcproLab/mdp/observations.py`, `arcproLab/mdp/terminations.py`
- Risk: Regression in RL performance or subtle bugs in error calculation (lateral/heading) that are hard to debug from training curves.
- Priority: High

**Asset Spawning:**
- What's not tested: Correctness of mass overrides and link properties after spawning.
- Files: `arcproLab/mdp/spawner.py`
- Risk: Robot might be simulated with default masses (30kg) if the override logic fails silently.
- Priority: Medium

## Missing Critical Features

**Stagnation Detection:**
- Problem: Policies can learn to sit still to avoid penalties if the speed reward is not balanced.
- Blocks: Efficient training and reliable resets.
- Files: `arcproLab/mdp/terminations.py` (stubbed `stagnation_termination`)

**Route Following:**
- Problem: `RoadGraph` is currently a random intent generator. It does not follow a predefined global path or mission.
- Blocks: Phase 12+ (Autonomous Navigation).
- Files: `arcproLab/mdp/road_graph.py`

---

*Concerns audit: 2024-04-14*

## Observed Behaviors (2026-05-05)

### Steering Geometry Mismatch (Ackermann vs Parallel)
- **Problem**: The `IsaacDirectEnv` uses an `AckermannComputer` to calculate different angles for inner/outer wheels. The `ManagerBasedRLEnv` (via `GroupedJointPositionAction`) forces both `Joint_Steer_L` and `Joint_Steer_R` to the exact same angle.
- **Impact**: Forcing parallel steering on a vehicle with Ackermann-aligned physical joints causes "scrubbing" and fighting against the PhysX solver. This is a primary suspect for the "wiggling wheels" and jitter observed during training.
- **Fix approach**: Implement an `AckermannJointPositionAction` in `mdp/actions.py` that replicates the `AckermannComputer` logic.

### Jitter and Steering Failure
- **Symptom**: Robot "wiggles" its wheels (high-frequency steering jitter) and fails at road bends.
- **Manual Verification**: Moving the robot forward manually works fine until a bend, but the policy fails to navigate curves.
- **Potential Causes**:
    - **Steering Gain**: The steering joint scale (1.0) might be too high for the 1.0x metric joints.
    - **Camera Alignment**: `TiledCamera` horizontal aperture or offset might be misaligned.
    - **CNN Convergence**: ResNet18 might not be extracting curve-specific features from 160x90 input.
