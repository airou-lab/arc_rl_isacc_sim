# Phase 16 History: MARL Transition & Environment Stabilization

## Summary
This phase focused on transitioning the ARCPro RL environment from a single-agent "Mastery" setup to a robust Multi-Agent Reinforcement Learning (MARL) foundation. The primary goal was to stabilize the environment at a true 1.0x metric scale with high-fidelity physics while resolving critical orientation and scaling issues that had accumulated during previous iterations.

## Findings & Bug Fixes

### 1. Robot Orientation Inversion
- **Issue**: The robot was spawning South-facing ( -1.5708 rad), causing immediate confusion for the `RoadGraph` and `TrackManager` which expected a North-facing start for the clockwise lap.
- **Fix**: Corrected the spawn rotation in `arcpro_env_cfg.py` to North-facing (1.5708 rad).
- **Code Reference**:
```python
# Face North (Corrected from South)
base_spawn_yaw = 1.5708 
```

### 2. Metric Scaling Synchronization
- **Issue**: Persistent confusion between "8x authoring scale" and "1x simulation scale". 
- **Fix**: Confirmed track is 1.0x native metric. The asset `no_graph_sim_clean_1x_flattened.usda` was verified to have `metersPerUnit = 1`. Configured simulation to use 1.0x scale for both track and robot to ensure physical consistency (mass, friction, torque).
- **Technical Note**: The 0.125 scale factor was identified as the necessary transformation to bring authoring units into metric meters.

### 3. Spawn Stability & Anti-Wheelie Tuning
- **Issue**: High-torque resets caused the robot to "wheelie" or flip upon spawning, leading to immediate termination.
- **Fix**: Lowered spawn height to 4cm (`hit["position"][2] + 0.04`), disabled initial randomized velocity (`rand_vel_x = 0`), and tuned throttle damping to 1.0 in `arcpro_robot_cfg.py`.
- **Code Reference**:
```python
# arcpro_robot_cfg.py
"throttle": ImplicitActuatorCfg(
    ...,
    damping=1.0, 
    armature=0.01,
)
```

### 4. Gate & Boundary Visualization
- **Issue**: Lane gates and boundaries were logically present but visually missing or incorrectly sized.
- **Fix**: Implemented `VisualizationMarkers` in `TrackManager` to render precision 0.05m spheres at all 24 gate locations and boundary points (subsampled for performance).
- **Code Reference**:
```python
def create_v(path, color, radius=0.05):
    cfg = VisualizationMarkersCfg(prim_path=path, markers={"dot": sim_utils.SphereCfg(radius=radius, ...)})
    return VisualizationMarkers(cfg)
```

### 5. Reward Tuning: Breaking the Suicide Bias
- **Issue**: Extreme stationary penalties were causing robots to choose immediate termination over standing still.
- **Fix**: Relaxed the stationary penalty to a weight of 1.0 (at < 0.1 m/s) to maintain forward pressure without triggering "suicide bias".

### 6. Performance Optimization
- **Optimization**: Tuned the vectorized environment count to 8-16. This provides a balance between high-throughput parallel samples and maintaining a high FPS/VRAM safety on consumer GPUs (RTX 3060), especially with HD cameras active.

## Technical Reference: High-Speed Physics
- **Simulation Frequency**: 500Hz (`dt=0.002`)
- **Control Frequency**: 50Hz (`decimation=10`)
- **Action Scale**: `-40.0` for combined drive.
- **Termination Threshold**: 0.15m from white/yellow road markings (`roadmark_contact`).

---
*History recorded: 2026-05-20*


## Fresh Training Restart & Physics Stabilization

**Context**: Discovered that the previous checkpoint was "poisoned" (trained while the USD asset was upside-down).
**Action**: Purged all old logs/ppo checkpoints and launched a completely fresh training run from Step 0.

**Physics Stabilization Fixes Applied**:
1. **Robot Spawn & Orientation**:
   Robot spawn is now perfectly flat at 12cm height with zero pitch. A North-facing WXYZ quaternion is applied via math to ensure alignment with the road graph.

2. **Rolling Start**:
   Enabled a rolling start (0.2 - 0.5 m/s) to overcome the strict 10-step stagnation limit right at the start of episodes.

3. **Contact Threshold Relaxation**:
   roadmark_contact threshold was relaxed from 0.15m to 0.05m to give the uninitialized ("drunk") agent a physical buffer against the narrow walls.

4. **Drive Scale**:
   drive.scale set to positive 20.0.

**Current Status**: 
The fresh run is active in the background. It is currently in the expected "Drunk Driving" phase (ep_len_mean ~8 steps, std ~1.0). The uninitialized neural network is applying maximum random steering into the narrow walls. This is completely normal for Step 0 and should improve over the next 500k steps.
