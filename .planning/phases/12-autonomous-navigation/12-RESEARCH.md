# Phase 12: Autonomous Navigation (RoadGraph Triggers) - Research

**Researched:** 2024-05-04
**Domain:** Autonomous Navigation & RoadGraph
**Confidence:** HIGH

## Summary

This phase implements trigger-based decision logic in `arcproLab/mdp/road_graph.py` to provide navigation tokens (Left/Straight/Right) to the policy. The logic relies on detecting proximity to intersection gates (Stop Lines) and extracting the `SignalTurnRelation` attribute from the USD stage.

**Primary recommendation:** Extend `RoadGraph` to parse `DSLaneGate` prims once at initialization and use a vectorized `torch.cdist` lookup in the `update()` loop to set `turn_token` based on the nearest gate's relation.

## User Constraints (from CONTEXT.md)

### Locked Decisions
- Logic must reside in `arcproLab/mdp/road_graph.py`.
- Must handle 24 gates detected in the `no_graph_sim_clean_1x.usda` scene.
- Observations must carry tokens to the policy (already supported in `observations.py`).
- Must not break the "Loop" mission (default to Straight when no trigger is active).

## Standard Stack

### Core
| Library | Version | Purpose | Why Standard |
|---------|---------|---------|--------------|
| torch | 2.x | Vectorized Math | Native for Isaac Lab and Policy Stack. |
| pxr (USD) | 23.11 | Scene Parsing | Standard for NVIDIA Omniverse/Isaac Sim. |

### Supporting
| Library | Version | Purpose | When to Use |
|---------|---------|---------|--------------|
| numpy | 1.24+ | Data Handling | Used for initial coordinate extraction. |

## Architecture Patterns

### Recommended Project Structure
```
arcproLab/mdp/
├── road_graph.py          # Implement trigger logic here
└── track_manager.py       # Source of gate positions (optional dependency)
```

### Pattern: Proximity-Based Intent Trigger
**What:** A spatial trigger that activates when the robot is within a specific radius of a gate.
**When to use:** When the high-level mission is known (e.g., "Loop") and the track layout provides explicit decision points (Gates).
**Example Logic:**
1. **At Start:** Find all `DSLaneGate` prims. Store `(X, Y)` and `RelationValue`.
2. **Each Step:** 
   - Compute `dist = min(dist(robot, all_gates))`.
   - If `dist < 2.5m`: `turn_token = nearest_gate.relation`.
   - Else: `turn_token = 0.0` (Straight).

## Gate Mapping Strategy

| USD Relation | Token Value | Description |
|--------------|-------------|-------------|
| "Straight"   | `0.0`       | Maintain current lane/heading through junction. |
| "Left"       | `-1.0`      | Prepare for left turn. |
| "Right"      | `1.0`       | Prepare for right turn. |

**Observation Vector Integration:**
The token is passed to `obs[:, 0]` in `get_telemetry_vector`. The policy uses this to adjust its internal steering/speed behavior.

## Don't Hand-Roll

| Problem | Don't Build | Use Instead | Why |
|---------|-------------|-------------|-----|
| Nearest Neighbor | Loop through list | `torch.cdist` | Massively parallel on GPU; handles 32+ environments efficiently. |
| Coordinate Transform | Custom Math | `UsdGeom.Xformable` | Handles nested scaling and offsets correctly. |

## Common Pitfalls

### Pitfall 1: Coordinate Scaling
**What goes wrong:** The track is scaled by `0.125` in `arcpro_env_cfg.py`. Extracting gate positions from the raw USD might ignore this scale.
**How to avoid:** Use `ComputeLocalToWorldTransform` on the prim after it has been spawned, or relative to the environment origin (consistent with `TrackManager`).

### Pitfall 2: Flapping Decisions
**What goes wrong:** At the edge of the 2.5m trigger, the decision might flicker.
**How to avoid:** Implement a small hysteresis or simply rely on the fact that the robot is moving towards the gate, so it will stay within the radius once it enters.

### Pitfall 3: Gate Misalignment
**What goes wrong:** Multiple gates at one junction are close together. The robot might trigger the "Left" gate while in the "Straight" lane.
**How to avoid:** The gates are usually placed *in* the specific lanes. As long as the robot follows the lane, it will be significantly closer to its own lane's gate.

## Code Examples

### Parsing Gates with Relations
```python
from pxr import Usd, UsdGeom
import torch

def get_gate_map(stage, env_origin):
    gate_data = []
    for prim in Usd.PrimRange(stage.GetPseudoRoot()):
        if prim.HasAttribute("primvars:ds_type"):
            if prim.GetAttribute("primvars:ds_type").Get() == ["DSLaneGate"]:
                # Get World Position
                xform = UsdGeom.Xformable(prim)
                world_transform = xform.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
                pos = world_transform.ExtractTranslation()
                
                # Get Relation
                rel = prim.GetAttribute("SignalTurnRelation").Get()
                val = 0.0 # Default
                if rel == "Left": val = -1.0
                elif rel == "Right": val = 1.0
                
                # Adjust for environment origin
                gate_data.append([pos[0] - env_origin[0], pos[1] - env_origin[1], val])
    
    return torch.tensor(gate_data, device="cuda:0")
```

## Validation Architecture

### Test Framework
| Property | Value |
|----------|-------|
| Framework | pytest |
| Quick run command | `pytest arcproLab/policy_stack/test_all_so_far.py` |

### Phase Requirements → Test Map
| Req ID | Behavior | Test Type | Automated Command |
|--------|----------|-----------|-------------------|
| NAV-TR-01 | Token is 0.0 when far from gates | Unit | `python3 arcproLab/scripts/verify_policy.py --debug` |
| NAV-TR-02 | Token changes to Relation when < 2.5m | Unit | `python3 arcproLab/scripts/verify_policy.py --debug` |

## Sources

### Primary (HIGH confidence)
- `arcproLab/mdp/road_graph.py` - Current implementation placeholder.
- `arcproLab/mdp/observations.py` - Verified telemetry mapping.
- `dump_gates_env.py` (Ad-hoc) - Confirmed 24 gates and their relations in the scene.

## Metadata

**Confidence breakdown:**
- Standard stack: HIGH
- Architecture: HIGH
- Pitfalls: MEDIUM

**Research date:** 2024-05-04
**Valid until:** 2024-06-04
