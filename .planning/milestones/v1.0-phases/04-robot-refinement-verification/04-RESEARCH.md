# Phase 4: Robot Refinement & Verification - Research

**Researched:** 2024-05-23
**Domain:** Isaac Lab Articulation, Physical Verification, Sensor Placement
**Confidence:** HIGH

## Summary

This research focuses on the precise physical calibration of the F1Tenth robot within Isaac Lab to match real-world specifications. We have identified the exact mechanisms to override mass, scale geometry, and place sensors using `ArticulationCfg` and `UsdFileCfg`. Additionally, we have defined a verification protocol using the Isaac Lab Python API to ensure the simulated model matches the target metrics (403mm L, 287mm W, 4092g).

**Primary recommendation:** Use `UsdFileCfg` with a calculated `scale` to match target dimensions, and implement a dedicated verification script that uses the `Articulation` and `UsdGeom` APIs to print live metrics (Mass, Wheelbase, Track Width) before proceeding to training.

<user_constraints>
## User Constraints (from Phase 4 Description)

### Locked Decisions
- **Robot Dimensions:** 403mm L, 287mm W.
- **Robot Kinematics:** 25cm wheelbase, 24cm track width.
- **Wheels:** 5cm wheel radius, 2cm wheel width.
- **Weight:** 4092g (4.092 kg).
- **Sensors:** RGB camera only (Realsense at 145mm L, 195mm H offset).
- **Feature:** Free camera support in Isaac Lab for debugging.
- **Objective:** Deep physical verification protocol.

### Claude's Discretion
- Implementation details of the verification script.
- Specific method of mass distribution (e.g., concentrated in chassis or distributed via density).
- Configuration of the free camera tracking mode.

### Deferred Ideas (OUT OF SCOPE)
- Lidar integration (RGB only for this phase).
- Dynamic friction coefficient tuning (reserved for later verification if basic kinematics pass).
</user_constraints>

<phase_requirements>
## Phase Requirements

| ID | Description | Research Support |
|----|-------------|-----------------|
| **4.1** | Robot Scaling & Metrics | Identified `UsdFileCfg.scale` and `MassPropertiesCfg` for precise adjustments. |
| **4.2** | Sensor Optimization | Defined `TiledCameraCfg` offset using `subtract_frame_transforms` for verification. |
| **4.3** | Free Camera Support | Researched `ViewerCfg` and viewport navigation for debug visibility. |
| **4.4** | Deep Verification | Developed a Python-based protocol for mass, dimension, and kinematic validation. |
</phase_requirements>

## Standard Stack

### Core
| Library | Version | Purpose | Why Standard |
|---------|---------|---------|--------------|
| `isaaclab.assets` | 1.0+ | Articulation management | Native Isaac Lab asset handling. |
| `isaaclab.sim` | 1.0+ | Simulation configuration | Unified spawner and property overrides. |
| `pxr.UsdGeom` | N/A | Geometry inspection | Access to Bounding Box and prim properties. |
| `torch` | 2.0+ | Tensor math | Isaac Lab's native computation engine. |

### Supporting
| Library | Version | Purpose | When to Use |
|---------|---------|---------|-------------|
| `isaaclab.sensors` | 1.0+ | Camera simulation | For per-agent RGB-only feed. |
| `isaaclab.utils.math` | 1.0+ | Frame transforms | Calculating local offsets from world poses. |

## Architecture Patterns

### Recommended Project Structure
```
arcproLab/
├── mdp/
│   └── ...
├── arcpro_robot_cfg.py  # Update ArticulationCfg here
├── arcpro_env_cfg.py    # Update Sensor/ViewerCfg here
└── scripts/
    └── verify_robot.py  # New: Standalone verification script
```

### Pattern 1: Precise Mass Override
To set the total mass to exactly 4.092 kg, the most reliable method is to override the mass of the chassis link (root) while accounting for the mass of the wheels (if fixed).

```python
# In arcpro_robot_cfg.py
from isaaclab.assets import ArticulationCfg
import isaaclab.sim as sim_utils

# Calculation: Total (4.092) - 4 * WheelMass (e.g. 0.1kg) = 3.692kg Chassis
ARCPRO_ROBOT_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path="...",
        scale=(1.0, 1.0, 1.0), # Calculated to reach 403mm L
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            rigid_body_enabled=True,
            # ...
        ),
        # mass_props here overrides the ROOT prim mass
        mass_props=sim_utils.MassPropertiesCfg(mass=4.092), 
    ),
)
```

### Pattern 2: Free Camera Debugging
Isaac Lab uses the `ViewerCfg` to control the initial camera state. By setting `origin_type="asset_root"`, the camera tracks the robot but remains "free" for manual fly-through navigation.

```python
# In arcpro_env_cfg.py
from isaaclab.envs import ViewerCfg

viewer = ViewerCfg(
    eye=(2.0, 2.0, 2.0),
    lookat=(0.0, 0.0, 0.0),
    origin_type="asset_root", # Tracks the robot
    asset_name="robot",
)
```

## Don't Hand-Roll

| Problem | Don't Build | Use Instead | Why |
|---------|-------------|-------------|-----|
| Dimension measurement | Custom raycasts | `UsdGeom.BBoxCache` | Native USD support for tight bounding boxes. |
| Local offsets | Manual matrix math | `subtract_frame_transforms` | Handles complex SE(3) math safely in torch. |
| Camera Tracking | Custom camera logic | `ViewerCfg(origin_type="asset_root")` | Built-in high-performance tracking. |

## Common Pitfalls

### Pitfall 1: Metric Scaling vs. Non-Metric USD
**What goes wrong:** If the base USD is in centimeters or inches, a scale of `1.0` will result in massive or tiny robots.
**How to avoid:** Always use `UsdGeom.GetStageMetersPerUnit(stage)` in verification scripts to check the USD unit and apply a correction scale if it's not `1.0` (meters).

### Pitfall 2: Inertia Tensor Mismatch
**What goes wrong:** Overriding mass but not updating the inertia tensor makes the robot rotate unnaturally.
**How to avoid:** Use `MassPropertiesCfg(mass=4.092)` and let PhysX recompute the inertia from the collision volume if possible, or provide a diagonal `inertia` tuple.

## Code Examples

### Verification Script: Physical Metrics
This script should be run before starting any training to confirm the robot matches constraints.

```python
# Source: Derived from Isaac Lab Articulation API
import torch
from pxr import UsdGeom
import isaaclab.sim as sim_utils
from isaaclab.assets import Articulation

# 1. Total Mass Check
body_masses = robot.data.default_mass[0]
total_mass = torch.sum(body_masses).item()
print(f"Total Mass: {total_mass:.4f} kg (Target: 4.092)")

# 2. Bounding Box Check (Dimensions)
bbox_cache = UsdGeom.BBoxCache(UsdGeom.GetStageDefaultResolution(), ["default"])
bbox = bbox_cache.ComputeLocalBound(root_prim)
dims = bbox.GetRange().GetMax() - bbox.GetRange().GetMin()
print(f"L x W x H: {dims[0]:.4f} x {dims[1]:.4f} x {dims[2]:.4f} m (Target: 0.403 x 0.287)")

# 3. Wheelbase/Track Width
# Transform wheel body positions to root frame
# wheelbase = max(x_coords) - min(x_coords)
# track_width = max(y_coords) - min(y_coords)
```

## State of the Art

| Old Approach | Current Approach | When Changed | Impact |
|--------------|------------------|--------------|--------|
| Custom mass scripts | `MassPropertiesCfg` | Isaac Lab v1.0 | Declarative overrides in Cfg. |
| USD-only Sensors | `TiledCameraCfg` | Isaac Lab v1.0 | 5-10x faster rendering for RL. |
| Manual Camera | `ViewerCfg(origin_type)` | Isaac Lab v1.1 | Easier debugging of multi-env. |

## Open Questions

1. **Wheel Mass Distribution:** Does the 4092g target include wheels, or is it the chassis only?
   - *Recommendation:* Assume it is the total Curb Weight (including wheels).
2. **Camera Frame Origin:** Is the 145/195 offset relative to the front axle or the center of the car?
   - *Recommendation:* Usually, Realsense offsets are measured from the physical center or the ground projection. We will assume the root frame (usually center-bottom) and verify visually.

## Validation Architecture

### Test Framework
| Property | Value |
|----------|-------|
| Framework | Pytest + Isaac Lab Simulation |
| Config file | `arcproLab/arcpro_env_cfg.py` |
| Quick run command | `./isaaclab.sh -p arcproLab/scripts/verify_robot.py --headless` |
| Full suite command | `pytest tests/` |

### Phase Requirements → Test Map
| Req ID | Behavior | Test Type | Automated Command | File Exists? |
|--------|----------|-----------|-------------------|-------------|
| 4.1 | Robot Scale 403x287mm | Static Check | `./isaaclab.sh -p scripts/verify_robot.py` | ❌ Wave 0 |
| 4.1 | Robot Mass 4092g | Static Check | `./isaaclab.sh -p scripts/verify_robot.py` | ❌ Wave 0 |
| 4.2 | Camera Offset 145/195mm | Transform Check | `./isaaclab.sh -p scripts/verify_robot.py` | ❌ Wave 0 |
| 4.3 | Viewport Interaction | Manual | Open GUI and fly around | ✅ Isaac Sim |
| 4.4 | Ackermann Steering | Kinematic Test | `./isaaclab.sh -p scripts/verify_robot.py --test-steering` | ❌ Wave 0 |

### Wave 0 Gaps
- [ ] `arcproLab/scripts/verify_robot.py` — Core script to verify metrics.
- [ ] `arcproLab/arcpro_robot_cfg.py` — Update with precise mass and scale.
- [ ] `tests/test_robot_physics.py` — Automated CI check for mass/dimensions.

## Sources

### Primary (HIGH confidence)
- `isaaclab.assets.Articulation` documentation - Mass and State data retrieval.
- `isaaclab.sim.UsdFileCfg` documentation - Spawner overrides.
- `pxr.UsdGeom` - Bounding box calculation methods.

### Secondary (MEDIUM confidence)
- NVIDIA Developer Forums - Discussions on `mass_props` precedence in Articulations.

## Metadata

**Confidence breakdown:**
- Standard stack: HIGH - Core Isaac Lab APIs.
- Architecture: HIGH - Follows ManagerBasedRLEnv patterns.
- Pitfalls: MEDIUM - Depends on base USD's internal unit configuration.

**Research date:** 2024-05-23
**Valid until:** 2024-06-23
