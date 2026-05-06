# Phase 14-01: USD Asset Flattening - Research

**Researched:** 2024-05-24
**Domain:** USD Geometry, Physical Scaling, RL Termination Calibration
**Confidence:** HIGH

## Summary

The current simulation environment uses a `scale=0.125` hack at the root of the USD stage to reconcile a 1.0x metric robot (0.3m wide) with a track asset that was likely authored at 8x scale (where 1 unit = 0.125m). This leads to several issues:
1.  **Metric Confusion:** Physics parameters and offsets must be constantly divided/multiplied by 0.125.
2.  **Strict Termination:** The `white_line_contact` termination uses a 0.15m threshold. In a 0.4375m wide lane (3.5m / 8), a 0.3m wide robot has a distance from center to line of 0.218m. A 0.15m threshold from center means the robot center can only move 6.8cm before termination—essentially zero tolerance for a high-speed RL agent.

The goal is to physically scale the geometry (vertices and translations) by 0.125 in the USD file and remove the root scale hack.

**Primary recommendation:** Use the `pxr.Usd` API to traverse the stage, scale all `points`, `widths`, `extents`, and `translations` by 0.125, and reset the `/World` scale to `(1, 1, 1)`.

## Standard Stack

### Core
| Library | Version | Purpose | Why Standard |
|---------|---------|---------|--------------|
| `pxr.Usd` | Latest (Isaac Sim) | USD Stage Manipulation | Official Pixar USD API |
| `pxr.UsdGeom` | Latest (Isaac Sim) | Geometry Schemas | Handles Mesh/BasisCurves/Xformable |
| `isaaclab.app.AppLauncher` | Latest | Environment Setup | Sets up library paths for USD/Omniverse |

### Supporting
| Library | Version | Purpose | When to Use |
|---------|---------|---------|--------------|
| `numpy` | Latest | Array manipulation | Useful for batch scaling points |

## Architecture Patterns

### Recommended Physical Scaling Pattern
To "flatten" a scale, the transformation must be pushed down to the leaf geometry (vertices) and intermediate translations.

1.  **Traverse Stage:** Iterate through all prims using `stage.Traverse()`.
2.  **Scale Geometry:** For `UsdGeom.Mesh` and `UsdGeom.BasisCurves`, multiply `points` attribute by the scale factor.
3.  **Scale Metadata:** Update `extent` and `widths` (for curves).
4.  **Scale Transforms:** For all `UsdGeom.Xformable`, scale the `translate` XformOp values.
5.  **Reset Root:** Set the root `scale` XformOp to `(1, 1, 1)`.

### Recalibration Pattern: "Leniency Buffer"
In RL, it is common to allow the robot's footprint to slightly overlap the boundary line before resetting.
- **Strict (Current):** `threshold = robot_half_width + epsilon` (e.g., 0.14 + 0.01 = 0.15m).
- **Leniency (Proposed):** `threshold = robot_half_width - overlap` (e.g., 0.14 - 0.02 = 0.12m).
This allows the agent to "ride the line" which is critical for high-speed racing lines.

## Don't Hand-Roll

| Problem | Don't Build | Use Instead | Why |
|---------|-------------|-------------|-----|
| USD Transformation | String replacement in USDA | `pxr.Usd` API | USDA is complex; manual string edits break references and binary USD compatibility. |
| Distance Calculation | Manual Point-to-Mesh | `torch.cdist` | highly optimized for GPU-accelerated batch distance checks. |

## Runtime State Inventory

| Category | Items Found | Action Required |
|----------|-------------|------------------|
| Stored data | `arcproLab/mdp/track_boundaries_1x.npz` | **Delete**. Must be regenerated to reflect new physical scale. |
| Stored data | `arcproLab/mdp/track_centerline_1x.npy` | **Regenerate**. Waypoints must match the new world coordinates. |
| Secrets/env vars | None | Verified — purely geometry change. |
| Build artifacts | None | Verified — code/asset change only. |

## Common Pitfalls

### Pitfall 1: Extent Blindness
**What goes wrong:** Objects disappear or flicker when the camera moves.
**Why it happens:** The `extent` attribute in USD is used for bounding box calculations and culling. If vertices are scaled but `extent` is not, the engine thinks the object is in its old position/size.
**How to avoid:** Always scale the `extent` attribute along with `points`.

### Pitfall 2: Double Scaling
**What goes wrong:** Objects become 64x smaller (0.125 * 0.125).
**Why it happens:** Applying the scale to `points` AND leaving the `scale` op on the parent.
**How to avoid:** Reset the root `scale` op to `(1, 1, 1)` in the same script.

### Pitfall 3: Stale Waypoints
**What goes wrong:** The robot drives into walls or "ghost" tracks.
**Why it happens:** `track_centerline_1x.npy` contains world coordinates for the policy target. If the USD moves or scales, these coordinates are invalid.
**How to avoid:** Run `generate_track.py` immediately after flattening the asset.

## Code Examples

### USD Scaling Script (Verified Pattern)
```python
from isaaclab.app import AppLauncher
import argparse

# Setup AppLauncher for USD paths
parser = argparse.ArgumentParser()
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
launcher = AppLauncher(args_cli)
simulation_app = launcher.app

from pxr import Usd, UsdGeom, Gf

def scale_asset(input_path, output_path, scale=0.125):
    stage = Usd.Stage.Open(input_path)
    for prim in stage.Traverse():
        # Scale Geometry
        if prim.IsA(UsdGeom.PointBased):
            geom = UsdGeom.PointBased(prim)
            pts = geom.GetPointsAttr().Get()
            if pts:
                geom.GetPointsAttr().Set([p * scale for p in pts])
            ext = geom.GetExtentAttr().Get()
            if ext:
                geom.GetExtentAttr().Set([e * scale for e in ext])
        
        # Scale Translations
        if prim.IsA(UsdGeom.Xformable):
            xf = UsdGeom.Xformable(prim)
            for op in xf.GetOrderedXformOps():
                if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                    val = op.Get()
                    if val: op.Set(val * scale)
                elif op.GetOpType() == UsdGeom.XformOp.TypeScale and prim.GetPath() == "/World":
                    op.Set(Gf.Vec3f(1.0, 1.0, 1.0))

    stage.Export(output_path)

scale_asset("openStreetUSD/no_graph_sim_clean_1x.usda", "openStreetUSD/no_graph_sim_clean_1x_flattened.usda")
simulation_app.close()
```

## Environment Availability

| Dependency | Required By | Available | Version | Fallback |
|------------|------------|-----------|---------|----------|
| Isaac Lab | Physics/USD API | ✓ | Latest | — |
| python.sh | Executing scripts | ✓ | 3.10 | — |
| openStreetUSD/no_graph_sim_clean_1x.usda | Source asset | ✓ | 62MB | — |

## Validation Architecture

### Test Framework
| Property | Value |
|----------|-------|
| Framework | custom (verify_metric.py) |
| Quick run command | `./verify_sim_metric.sh` |
| Full suite command | `pytest tests/test_track_manager.py` |

### Phase Requirements → Test Map
| Req ID | Behavior | Test Type | Automated Command |
|--------|----------|-----------|-------------------|
| USD-01 | USD points scaled | unit | `grep "points = " ..._flattened.usda` (manual point check) |
| CFG-01 | Env cfg points to 1x | smoke | `grep "scale=(1.0, 1.0, 1.0)" arcpro_env_cfg.py` |
| TERM-01 | Lenient termination | integration | `python3 arcproLab/scripts/verify_metric.py --headless` |

## Sources
### Primary (HIGH confidence)
- `openStreetUSD/no_graph_sim_clean_1x.usda`: Observed root scale (0.125).
- `arcproLab/mdp/track_manager.py`: Identified hardcoded cache paths and marker extraction logic.
- `arcproLab/mdp/terminations.py`: Found `0.15` threshold for white line contact.
- `arcproLab/arcpro_env_cfg.py`: Verified robot track width (0.28m) and existing (partially updated) asset config.

## Metadata
**Confidence breakdown:**
- Standard stack: HIGH - standard Isaac Sim/USD.
- Architecture: HIGH - straightforward flattening.
- Pitfalls: HIGH - cache invalidation is the biggest risk.

**Research date:** 2024-05-24
**Valid until:** 2024-06-24
