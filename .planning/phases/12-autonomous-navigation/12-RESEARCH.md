# Phase 12: Autonomous Intersection Navigation - Research

**Researched:** 2025-05-15
**Domain:** V2I Decentralized Coordination & USD Topology
**Confidence:** HIGH

## Summary
This research defines the implementation strategy for Task 12-02 (Edge Module Spawning). We shift from a centralized RoadGraph to a decentralized V2I architecture where "Smart Junction Modules" are spawned based on USD `DSIntersection` prims. Each module handles its own FSM state and robot handshakes.

**Primary recommendation:** Use `torch.cdist` for high-performance spatial lookup of junctions and implement a `JunctionModule` class that validates the robot's `PlanarPath` against its local intersection topology.

<user_constraints>
## User Constraints (from CONTEXT.md)

### Locked Decisions
- Use Decentralized V2I (Edge Modules) instead of a central planner.
- Intersections must be physically tied to `DSIntersection` prims.
- Robots must provide `PlanarPath` to the module for validation.

### the agent's Discretion
- Implementation of spatial lookup (Hashing vs. Tensors).
- Junction Module internal FSM state logic.
- PlanarPath validation heuristics.

</user_constraints>

<phase_requirements>
## Phase Requirements

| ID | Description | Research Support |
|----|-------------|------------------|
| NAV-01 | USD Topology Mapping | Analysis of `DSIntersection` and `DSLaneGate` attributes provides the linking mechanism. |
| NAV-02 | Edge Module Spawning | Class structure for `JunctionModule` and `JunctionManager` defined. |
| NAV-03 | Decentralized Lookup | `torch.cdist` confirmed as high-performance solution. |
| NAV-04 | Path Handshake | `PlanarPath` integration with `JunctionModule` handshake protocol defined. |
</phase_requirements>

## Standard Stack

### Core
| Library | Version | Purpose | Why Standard |
|---------|---------|---------|--------------|
| torch | 2.10.0+cu128 | Spatial math & Vectorization | Required by Isaac Lab and Policy Stack. |
| pxr (USD) | 23.11 | USD Scene Parsing | Standard for NVIDIA Omniverse/Isaac Sim. |
| numpy | 1.26.4 | Data manipulation | Supporting library for coordinate transforms. |

## Architecture Patterns

### Recommended Project Structure
```
arcproLab/mdp/
├── junction_manager.py    # Singleton managing all modules
├── junction_module.py     # Class definition for Edge Modules
└── road_graph.py          # Updated to delegate to JunctionManager
```

### Pattern: Smart Edge Module (V2I)
**What:** Each intersection is an independent actor with its own state.
**When to use:** Decentralized coordination scenarios.
**Example:**
```python
# Extracting Junctions from USD
intersections = {}
for prim in Usd.PrimRange(stage.GetPseudoRoot()):
    if prim.GetAttribute("primvars:ds_type").Get() == "DSIntersection":
        id = prim.GetAttribute("IntersectionID").Get()
        pos = prim.GetAttribute("xformOp:transform").Get().ExtractTranslation()
        gates = prim.GetAttribute("primvars:gates").Get() # List of GateIDs
        intersections[id] = JunctionModule(id, pos, gates)
```

## Don't Hand-Roll

| Problem | Don't Build | Use Instead | Why |
|---------|-------------|-------------|-----|
| Spatial Lookup | Custom KD-Tree | `torch.cdist` | Torch is already on GPU; cdist is highly optimized for M:N distances. |
| Path Planning | New Spline Gen | `PlanarPathPlanner` | Already implemented in `policy_stack` worker layer. |

## USD Topology Mapping (Implementation Details)

### Junction Extraction
Each `DSIntersection` prim in `openStreetUSD/no_graph_sim_clean_1x.usda` serves as a container for junction logic.
- **Coordinates**: Use `xformOp:transform` (matrix4d) or `AnalyticalPos` (float3) attribute to extract world-space XY.
- **Linking**: The `primvars:gates` attribute (type `string[]`) contains the `GateID`s of all associated `DSLaneGate` prims.
- **Logical Mapping**: `DSLaneGate` prims provide the `ODMapLaneID` (the lane segment it guards) and `SignalTurnRelation` (Straight/Left/Right).

## Junction Module Design

### Class Structure
```python
class JunctionModule:
    """Represents a standalone V2I edge module for a single intersection."""
    def __init__(self, junction_id: str, position: torch.Tensor, gates_metadata: dict):
        self.id = junction_id
        self.pos = position # World XY Tensor
        self.gates = gates_metadata # Dict: GateID -> {pos, lane_id, relation}
        self.state = "GREEN" # Simple FSM: GREEN, YELLOW, RED
        self.radius_trigger = 3.0
        self.radius_commit = 1.5
        
    def get_commands(self, robot_pos: torch.Tensor, intent_lane_id: str):
        """
        Calculates turn_token and go_signal.
        - Maps intent_lane_id to SignalTurnRelation.
        - Checks self.state for go_signal.
        """
        pass
```

## Decentralized Lookup

### High-Performance Spatial Query
- **Method**: Vectorized Distance Matrix.
- **Implementation**: 
    1. Collect all `JunctionModule.pos` into a single `(M, 2)` tensor.
    2. Compute `distances = torch.cdist(robot_pos[:, :2], junction_tensor)`.
    3. `nearest_idx = torch.argmin(distances, dim=1)`.
- **Complexity**: $O(N \times M)$ where $N$ is robots and $M$ is junctions. For $N=100, M=100$, this is negligible in Torch.

## Integration with PlanarPath

### Handshake Protocol
- **Input**: The `JunctionModule` accepts a `(5, 2)` tensor representing the `PlanarPath` from the policy's planner.
- **Validation**: 
    1. Check if the path's terminal waypoint matches the exit lane's gate position.
    2. (Future) Check for spatial overlaps with other agents' paths for collision avoidance.
- **Flow**: `observations.py` calls `JunctionManager.update(robot_pos, planar_paths)`, which delegates to the nearest modules.

## Common Pitfalls

### Pitfall 1: Coordinate System Mismatch
**What goes wrong:** USD coordinates extracted in Local Xform instead of World Xform.
**How to avoid:** Always use `UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())` to get the world matrix.

### Pitfall 2: Batch Size mismatch
**What goes wrong:** `JunctionManager` assumes a fixed number of robots.
**How to avoid:** Dynamically resize tensors if `env.num_envs` changes or use batch-aware broadcasting.

## Environment Availability

| Dependency | Required By | Available | Version | Fallback |
|------------|------------|-----------|---------|----------|
| Torch | Spatial Math | ✓ | 2.10.0+cu128 | — |
| PXR (USD) | Scene Parsing | ✓ | 23.11 | — |
| Isaac Lab | Simulation | ✓ | — | — |

## Sources

### Primary (HIGH confidence)
- `openStreetUSD/no_graph_sim_clean_1x.usda` - Verified `DSIntersection` and `DSLaneGate` structure.
- `arcproLab/mdp/track_manager.py` - Verified pattern for spatial tensor management.
- `arcproLab/mdp/observations.py` - Verified telemetry vector structure.

## Metadata
**Confidence breakdown:**
- Standard stack: HIGH - Core project dependencies.
- Architecture: HIGH - Follows existing Isaac Lab / ARCPro patterns.
- Pitfalls: MEDIUM - Based on common USD/Torch integration issues.

**Research date:** 2025-05-15
**Valid until:** 2025-06-15
