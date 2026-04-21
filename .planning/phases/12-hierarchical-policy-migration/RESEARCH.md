# Phase 12: Hierarchical Policy Migration - Research

**Researched:** 2026-03-25
**Domain:** Autonomous Driving / Reinforcement Learning / Hierarchical Control
**Confidence:** HIGH

## Summary

This phase involves porting the "Worker-Driver" hierarchical architecture from the existing reference stack to the new 1.0x metric Isaac Lab environment. The architecture decouples high-level route planning (Worker) from low-level visual-motor control (Driver/Policy). The research confirms that the core components (HPPP policy, Intersection Graph, Worker Scheduler) are already present in the `arcproLab/policy_stack/` directory and are largely compatible with the 1.0x scale, though some integration "glue" and geometry calibration for the new map are required.

**Primary recommendation:** Implement a vectorized `NavigationManager` within the Isaac Lab `mdp` layer to coordinate the `WorkerNode`s and provide the navigation intent signals (`turn_token`, `go_signal`) to the policy.

<user_constraints>
## User Constraints (from CONTEXT.md)

### Locked Decisions
- **Scale**: 1.0x Metric (True Physics).
- **Mode**: "Road in Void" (Grass/Fences ghosted).
- **Orientation**: **South-facing** (-1.57 rad).
- **Target**: Right Lane Center (+2.42m from Yellow Line).
- **Boundaries**: Strict Reset (Safe Zone: 0.225m to 2.5m).
- **Control Rate**: 20Hz (`decimation=25`).
- **Action Fix**: **Inverted Throttle** (Scale: -60.0).

### the agent's Discretion
- Hierarchical Path Planning Policy (HPPP) implementation details.
- Integration pattern for Worker-Driver architecture in Isaac Lab.

### Deferred Ideas (OUT OF SCOPE)
- Continuous RVO (Phase 2 RVO is temporal separation only).
- Dynamic traffic light integration (Phase 11/12 focus is on road topology first).
</user_constraints>

<phase_requirements>
## Phase Requirements

| ID | Description | Research Support |
|----|-------------|------------------|
| REQ-NAV-GRAPH | Road topology represented as a graph of nodes and edges. | `IntersectionGraph` and `intersection_topology.json` identified as core components. |
| REQ-NAV-TRANSITION | Seamless handover between road segments. | `WorkerNode` state machine (CRUISING -> DECIDING -> COMMITTED) manages transitions. |
| REQ-SIM-METRIC | 1.0x metric scale for robot and track. | `WAYPOINT_NORM_SCALE` and `WorkerScheduler` config verified for 1.0x metrics. |
</phase_requirements>

## Standard Stack

### Core
| Library | Version | Purpose | Why Standard |
|---------|---------|---------|--------------|
| `sb3_contrib` | Latest | `RecurrentPPO` | Supports LSTM-based policies needed for HPPP. |
| `torch` | 2.x | Tensor operations | Deep learning backend. |
| `numpy` | 1.2x | Classical planning | Efficient graph and coordination math. |

### Supporting
| Library | Version | Purpose | When to Use |
|---------|---------|---------|--------------|
| `IntersectionGraph` | Internal | Topology management | To resolve turns at intersections. |
| `WorkerScheduler` | Internal | Intersection coordination | Multi-agent "go/wait" logic. |

## Vectorization Strategy (Performance Optimization)
The current `IntersectionGraph` and `WorkerScheduler` are implemented with standard Python dictionaries and lists. To maintain 100+ FPS in Isaac Lab with 32+ robots, we must transition to a tensor-parallel approach.

### 1. Vectorized Intersection Triggers
- **Risk:** Looping through 32 environments and then looping through N intersections in Python is O(num_envs * N_intersections).
- **Strategy:** Convert all intersection positions (X, Y) into a single `torch.Tensor` of shape `(N_intersections, 2)`.
- **Logic:** Use `torch.cdist(robot_positions, intersection_positions)` to compute all distances in a single GPU call. Trigger the "Worker Decision" state using `torch.any(distances < trigger_radius)`.

### 2. Batch State Machine Update
- **Risk:** `WorkerNode` uses a complex state machine (CRUISING, DECIDING, COMMITTED).
- **Strategy:** Represent the state of all 32 workers as an integer tensor `worker_states = torch.zeros(32, dtype=torch.int32)`.
- **Logic:** Use `torch.where` and boolean masking to update states in parallel without Python `if/else` loops.

### 3. Scheduler Conflict Matrix
- **Risk:** Pairwise conflict checking in `WorkerScheduler` is slow for many robots.
- **Strategy:** Maintain an `active_intents` tensor and a `conflict_mask` pre-computed from the topology.
- **Logic:** Use bitwise operations on tensors to resolve GO/WAIT signals for all robots simultaneously.

## Architecture Patterns

### Recommended Project Structure
```
arcproLab/
├── policy_stack/             # Submodule (Reference Stack)
│   ├── policies/
│   │   └── hierarchical_policy.py
│   └── agent/
│       ├── agent_node.py
│       ├── intersection_graph.py
│       └── worker_scheduler.py
└── mdp/                      # Isaac Lab Integration Layer
    ├── navigation_manager.py # NEW: Vectorized Worker coordination
    ├── observations.py      # Update: Inject navigation intent
    └── actions.py           # Update: Safety gate in CombinedDriveAction
```

### Pattern: Worker-Driver Decoupling
**What:** The Worker (Graph Planner) provides a discrete `turn_token` {-1, 0, 1} and a `go_signal` {0, 1} to the Driver (Learned Policy).
**When to use:** Complex navigation tasks where end-to-end learning of route planning is inefficient.
**Example:**
```python
# Source: arcproLab/policy_stack/agent/agent_node.py
token, go = self.worker.step(position, heading, speed)
obs["vec"][0] = token
obs["vec"][1] = go
# Policy then uses token to bias kinematic anchors
```

### Anti-Patterns to Avoid
- **End-to-End Route Learning:** Forcing the CNN/LSTM to learn the map topology. This makes the policy fragile to map changes.
- **Continuous Collision Avoidance:** Implementing complex obstacle avoidance before basic intersection "go/wait" coordination is stable.

## Don't Hand-Roll

| Problem | Don't Build | Use Instead | Why |
|---------|-------------|-------------|-----|
| Route Planning | Custom A* | `IntersectionGraph` | Already handles topology and TurnCommand mapping. |
| Intersection Safety | Learned Braking | `AgentNode` Safety Gate | Hard safety overrides are required for multi-agent reliability. |
| Geometry Discovery | Manual USD measurement | `GeometryCalibrator` | Auto-discovers lengths/positions from PhysX directly. |

## Common Pitfalls

### Pitfall 1: Unit Mismatch (8x vs 1x)
**What goes wrong:** Policy expects 8x scale coordinates, resulting in massive over-steering or planning waypoints 20m ahead.
**How to avoid:** Ensure `WAYPOINT_NORM_SCALE=2.5` and `waypoint_horizon=2.5` in HPPP. Verify `WorkerScheduler.config.vehicle_length=0.33`.

### Pitfall 2: Discontinuous Turn Tokens
**What goes wrong:** Worker flips turn_token while agent is in the middle of a turn, causing erratic steering.
**How to avoid:** Use the `COMMITTED` state in `WorkerNode` to latch the token until the intersection is cleared.

### Pitfall 3: Missed Intersection Triggers
**What goes wrong:** Global position drift causes the robot to "miss" the 3m radius of an intersection.
**How to avoid:** Use `GeometryCalibrator` to ensure center positions are exact. For deployment, use `step_topological` with EKF arc-length.

## Code Examples

### Vectorized Navigation Manager (Draft)
```python
# To be implemented in Phase 12
class NavigationManager:
    def __init__(self, num_envs, graph, device):
        self.workers = [WorkerNode(f"env_{i}", graph) for i in range(num_envs)]
        self.scheduler = WorkerScheduler()
        
    def step(self, positions, headings, speeds, dt):
        tokens = torch.zeros(len(self.workers))
        signals = torch.zeros(len(self.workers))
        for i, (pos, hdg, spd) in enumerate(zip(positions, headings, speeds)):
            tokens[i], signals[i] = self.workers[i].step(pos, hdg, spd, dt, self.scheduler)
        return tokens, signals
```

## Environment Availability

| Dependency | Required By | Available | Version | Fallback |
|------------|------------|-----------|---------|----------|
| Isaac Sim | Simulation | ✓ | 4.x | — |
| `sb3_contrib` | Policy training | ✓ | — | — |
| `IntersectionGraph` | Navigation | ✓ | — | — |

## Validation Architecture

### Test Framework
| Property | Value |
|----------|-------|
| Framework | pytest |
| Config file | `arcproLab/policy_stack/pytest.ini` |
| Quick run command | `pytest arcproLab/policy_stack/test_all_so_far.py` |

### Phase Requirements → Test Map
| Req ID | Behavior | Test Type | Automated Command | File Exists? |
|--------|----------|-----------|-------------------|-------------|
| REQ-NAV-GRAPH | Graph loads from JSON | Unit | `pytest arcproLab/policy_stack/test_all_so_far.py` | ✅ |
| REQ-NAV-TRANSITION | Worker state machine transitions | Unit | `pytest arcproLab/policy_stack/test_all_so_far.py` | ✅ |
| REQ-SIM-METRIC | Waypoint scale is 1.0x | Unit | `pytest arcproLab/scripts/test_sb3_wrapper.py` | ✅ |

## Sources

### Primary (HIGH confidence)
- `arcproLab/policy_stack/` - Reference implementation code.
- `arcproLab/mdp/observations.py` - Current telemetry implementation.
- `arcproLab/arcpro_env_cfg.py` - Current Isaac Lab configuration.

## Metadata

**Confidence breakdown:**
- Standard stack: HIGH - Using established `policy_stack`.
- Architecture: HIGH - Worker-Driver is a proven pattern in this project.
- Pitfalls: MEDIUM - Scale transitions are always tricky.

**Research date:** 2026-03-25
**Valid until:** 2026-04-25
