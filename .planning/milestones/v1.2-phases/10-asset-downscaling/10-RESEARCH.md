# Phase 10: Asset Downscaling - Research

**Researched:** 2026-04-12
**Domain:** USD Manipulation, Isaac Sim Physics Scaling, Robotics Control
**Confidence:** HIGH

## Summary

Phase 10 focuses on reverting the simulation environment from an 8.0x "Giant Scale" workaround (implemented in Phase 9 for stability) back to a true 1.0x metric scale. This involves surgical modification of the USD road assets, downscaling waypoint data, and re-tuning the robot's physical properties (mass and actuator gains) to ensure stability at the original scale.

**Primary recommendation:** Use a standalone USD manipulation script based on `trash/tools/extract_road_gsd.py` to bake a 0.125x scale and isolation filters directly into a new `no_graph_sim_metric.usd` file, then update the environment configuration to use 1.0x scales and remove observation normalization.

## Standard Stack

### Core
| Library | Version | Purpose | Why Standard |
|---------|---------|---------|--------------|
| Isaac Sim / Lab | 4.2.0+ | Simulation Framework | Primary framework for the project. |
| pxr (USD) | Latest | Scene Manipulation | Industry standard for 3D scene description. |
| torch | 2.0+ | Observation Handling | Required for vectorized environment logic. |

### Supporting
| Library | Version | Purpose | When to Use |
|---------|---------|---------|-------------|
| numpy | Latest | Waypoint Storage | Used for loading/saving `.npy` track data. |

## Architecture Patterns

### USD Isolation & Scaling Pattern
To isolate specific prims and apply a global downscale, use the `pxr.Usd` API to traverse the stage and collect geometry into a new consolidated mesh. This "bakes" the scaling into the vertex positions, which is more robust for PhysX than applying a transform to a group.

**Example Logic:**
```python
# Based on trash/tools/extract_road_gsd.py
include_keywords = ["road", "pavement", "asphalt", "marker", "yellow", "white"]
factor = 0.125  # Downscale from 8x to 1x

for prim in source_stage.Traverse():
    if prim.IsA(UsdGeom.Mesh):
        path = prim.GetPath().pathString.lower()
        if any(k in path for k in include_keywords):
            # ... transform points and multiply by factor ...
```

### Observation Scaling Pattern
Observation normalization (e.g., `* 0.125`) should be removed entirely in `mdp/observations.py` when the simulation scale matches the policy's expected units (meters and m/s).

## Don't Hand-Roll

| Problem | Don't Build | Use Instead | Why |
|---------|-------------|-------------|-----|
| USD Transformation | Manual vertex edits | `UsdGeom.Xformable` | Handles complex hierarchy transforms correctly. |
| Waypoint Resampling | Manual coordinate math | `TrackManager.sample_waypoints_from_usd` | Automatically handles ordering and density using nearest-neighbor logic. |

## Runtime State Inventory

| Category | Items Found | Action Required |
|----------|-------------|------------------|
| Stored data | `arcproLab/mdp/track_centerline.npy` | **Data Migration**: Must be re-sampled from the new 1.0x USD or manually scaled by 0.125. |
| Live service config | None | N/A |
| OS-registered state | None | N/A |
| Secrets/env vars | None | N/A |
| Build artifacts | `trash/openStreetUSD/no_graph_sim.usda` | **None**: Use as reference only. |

## Common Pitfalls

### Pitfall 1: Small-Scale Jitter (PhysX)
**What goes wrong:** At 1.0x metric scale (robot ~0.3m), PhysX might experience jitter or "exploding" joints if time steps or solver iterations are too low.
**How to avoid:** 
- Keep `solver_type=1` (TGS).
- Use `max_position_iteration_count >= 8`.
- Set `dt <= 0.005` (200Hz).
- **Crucial:** Add `armature` (e.g., 0.01) to `ImplicitActuatorCfg` to add virtual inertia.

### Pitfall 2: Marker Collision Thresholds
**What goes wrong:** Hard-coded thresholds for "hitting a white line" (e.g., 4.0m in `mdp/terminations.py`) will reset the robot immediately at 1.0x scale.
**How to avoid:** Scale all termination thresholds by 0.125 (e.g., 4.0m becomes 0.5m).

## Code Examples

### 1. Actuator Gains for 20kg F1Tenth
Recommended values based on Isaac Lab mobile robot defaults:
```python
# arcproLab/arcpro_robot_cfg.py
actuators: dict = {
    "steering": ImplicitActuatorCfg(
        joint_names_expr=["Joint_Steer_.*"],
        stiffness=50.0,  # Reverted from 500k
        damping=2.0,     # Reverted from 10k
        armature=0.01,   # Stability helper
    ),
    "throttle": ImplicitActuatorCfg(
        joint_names_expr=["Joint_Drive_.*"], 
        stiffness=0.0,
        damping=5.0,     # Reverted from 5k
        armature=0.01,   # Stability helper
    ),
}
```

### 2. Scaled Observation Removal
```python
# arcproLab/mdp/observations.py
# BEFORE
obs[:, 3] = asset.data.root_lin_vel_b[:, 0] * 0.125
# AFTER
obs[:, 3] = asset.data.root_lin_vel_b[:, 0]  # True m/s
```

## State of the Art

| Old Approach | Current Approach | When Changed | Impact |
|--------------|------------------|--------------|--------|
| 8x Giant Scale | 1.0x Metric Scale | Phase 10 | Improves sim-to-real transferability and matches physical units. |
| Massive Gain Overrides | Realistic Gains + Armature | Phase 10 | Reduces numerical stiffness while maintaining stability. |

## Open Questions

1. **Why was 20kg chosen?**
   - What we know: F1Tenth is usually < 5kg. 20kg was specified in the requirements.
   - Recommendation: Follow the 20kg requirement but keep gains tunable in case it's too sluggish.

2. **Absolute Lane Width?**
   - What we know: Current "Giant" lane center is 2.25m from centerline.
   - The gap: Is 0.28m (1x scale) the *actual* metric width of the asset's lane?
   - Recommendation: Verify with `measure_dimensions.py` *after* downscaling.

## Environment Availability

| Dependency | Required By | Available | Version | Fallback |
|------------|------------|-----------|---------|----------|
| Isaac Sim | Simulation | ✓ | 4.2.0 | — |
| Python pxr | USD Ops | ✓ | Latest | — |

## Validation Architecture

### Test Framework
| Property | Value |
|----------|-------|
| Framework | pytest |
| Config file | `tests/` directory |
| Quick run command | `pytest tests/test_track_manager.py` |

### Wave 0 Gaps
- [ ] `scripts/measure_dimensions.py` update to check 1.0x targets.
- [ ] New test `tests/test_scaling.py` to verify robot dimensions and waypoint ranges.

## Metadata

**Confidence breakdown:**
- Standard stack: HIGH
- Architecture: HIGH
- Pitfalls: MEDIUM (Jitter depends on specific PhysX version)

**Research date:** 2026-04-12
**Valid until:** 2026-05-12
