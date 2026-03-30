# Requirements: Lane Follower & Graph Navigation

**Defined:** 2026-03-25
**Core Value:** Autonomous navigation between lanes and across intersections.

## v1.0 MVP (Completed)

- [x] **REQ-ENV-LANE**: Isaac Sim scene with two parallel lanes (visual or semantic).
- [x] **REQ-ROBOT-CAM**: Differential drive robot with a mounted RGB camera sensor.
- [x] **REQ-VIS-DETECT**: Logic to identify left and right lane boundaries from camera input.
- [x] **REQ-CTRL-CENTER**: PID or similar controller to maintain robot position between lanes.

## v1.1 Intersection & Graph Navigation (Phase 6)

- [ ] **REQ-NAV-GRAPH**: Road topology represented as a graph of nodes (intersections) and edges (road segments).
- [ ] **REQ-NAV-TRANSITION**: Seamless handover of robot tracking between consecutive or branching road segments at nodes.
- [ ] **REQ-INT-LIGHT**: Functional traffic light assets in the Isaac Sim scene at intersection nodes.
- [ ] **REQ-INT-CTRL**: ROS 2 control mechanism for traffic light states using USD Variants (Red, Yellow, Green).
- [ ] **REQ-INT-OBS**: Integration of traffic light states into the RL observation vector for the robot.
- [ ] **REQ-INT-REWARD**: Training rewards based on traffic light compliance (e.g., penalty for red light violations).

## v1.2 Simulation Fidelity (Phase 7)

- [ ] **REQ-SIM-METRIC**: Simulation uses 1.0x metric scale for both robot and track.
- [ ] **REQ-SIM-STABILITY**: Physics remain stable without damping/stiffness overrides.
- [ ] **REQ-SIM-TRANSFER**: Configuration is realistic enough for sim-to-real transfer.

## Traceability

| Requirement | Phase | Status |
|-------------|-------|--------|
| REQ-ENV-LANE | Phase 1 | COMPLETE |
| REQ-ROBOT-CAM | Phase 2 | COMPLETE |
| REQ-VIS-DETECT | Phase 3 | COMPLETE |
| REQ-CTRL-CENTER | Phase 4 | COMPLETE |
| REQ-NAV-GRAPH | Phase 6 | Planned |
| REQ-NAV-TRANSITION | Phase 6 | Planned |
| REQ-INT-LIGHT | Phase 6 | Planned |
| REQ-INT-CTRL | Phase 6 | Planned |
| REQ-INT-OBS | Phase 6 | Planned |
| REQ-INT-REWARD | Phase 6 | Planned |
| REQ-SIM-METRIC | Phase 7 | In Progress |
| REQ-SIM-STABILITY | Phase 7 | In Progress |
| REQ-SIM-TRANSFER | Phase 7 | In Progress |

---
*Requirements updated: 2026-03-25*
