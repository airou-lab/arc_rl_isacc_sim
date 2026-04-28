# Phase 12 Plan: Autonomous Intersection Navigation

This phase implements the high-level decision logic required for the robot to navigate the road network, moving from simple lane-following to graph-based routing.

## Goals
1. **The Turning Mission**: Prove that the HPPP policy correctly steers when the `turn_token` is flipped.
2. **Graph Discovery**: Automate the mapping of the USD road network into a topological adjacency list.
3. **Mission Execution**: Enable the robot to follow a sequence of turns to reach a destination.

---

## 1. Proof of Concept: Turning (Wave 0)
- [ ] **Task 12-01-turning-mission**: 
    - Identify a specific intersection gate in the `no_graph_sim_clean_1x.usda`.
    - Modify `RoadGraph.py` to hardcode a turn command (e.g., LEFT) when the robot is within 2.5m of that gate.
    - Verify in the GUI that the robot initiates the turn.

## 2. Edge Module Intelligence (Wave 1)
- [ ] **Task 12-02-edge-module-spawning**: 
    - Implement a USD parser that identifies all `DSIntersection` prims.
    - Create a `JunctionModule` class that manages state (Traffic Lights, Turn Grants) for a specific physical location.
    - Build a registry in `RoadGraph.py` that maps world-coordinates to local `JunctionModule` instances.
- [ ] **Task 12-03-handshake-logic**:
    - Update `observations.py` to perform a proximity-based "handshake" with the nearest `JunctionModule`.
    - Enable the module to dynamically override the robot's `turn_token` based on its location.
- [ ] **Task 12-04-waypoint-handover**:
    - Update `TrackManager` to swap waypoint segments when a `JunctionModule` signals a successful traversal.

## 3. Visual Diagnostics (Wave 2)
- [ ] **Task 12-04-planar-path-viz**:
    - Integrate the new `PlanarPathPlanner` from the `policy_stack` submodule.
    - Use `TrackManager` to visualize the 5-waypoint polyline in the Isaac Sim GUI.
    - Verify that the plan aligns with the actual turn taken by the robot.

---

## Verification Strategy
- **Turning Check**: Robot steers into an intersection and doesn't just hit the far wall.
- **Topology Check**: Print the discovered graph and verify it matches the visual USD layout.
- **Handover Check**: Robot continues lane-following on the target road *after* completing the turn.
