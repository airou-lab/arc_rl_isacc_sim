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
