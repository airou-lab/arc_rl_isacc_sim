# Requirements: Lane Follower & Graph Navigation

**Defined:** 2026-03-25
**Core Value:** Autonomous navigation between lanes and across intersections.

## v1.0 MVP (Completed)

- [x] **REQ-ENV-LANE**: Isaac Sim scene with two parallel lanes (visual or semantic).
- [x] **REQ-ROBOT-CAM**: Differential drive robot with a mounted RGB camera sensor.
- [x] **REQ-VIS-DETECT**: Logic to identify left and right lane boundaries from camera input.
- [x] **REQ-CTRL-CENTER**: PID or similar controller to maintain robot position between lanes.

## v2.0 Intersection & Graph Navigation (Completed)

- [x] **REQ-NAV-GRAPH**: Road topology represented as a graph of nodes (intersections) and edges (road segments).
- [x] **REQ-NAV-TRANSITION**: Seamless handover of robot tracking between consecutive or branching road segments at nodes.
- [x] **REQ-INT-LIGHT**: Functional traffic light assets in the Isaac Sim scene at intersection nodes.
- [x] **REQ-INT-CTRL**: Control mechanism for traffic light states using USD Variants.
- [x] **REQ-INT-OBS**: Integration of traffic light states into the RL observation vector for the robot.
- [x] **REQ-INT-REWARD**: Training rewards based on traffic light compliance.

## v2.5 HD Perception (Active)

- [x] **REQ-VIS-HD**: Transition to 640x360 high-resolution vision.
- [x] **REQ-BACKBONE-ADAPTIVE**: VRAM-efficient CNN backbone utilizing early pooling.
- [ ] **REQ-PROD-HARDENING**: Successful 5M step training run with zero target offset.

## v1.2 Simulation Fidelity (Completed)

- [x] **REQ-SIM-METRIC**: Simulation uses 1.0x metric scale for both robot and track.
- [x] **REQ-SIM-STABILITY**: Physics remain stable without damping/stiffness overrides.
- [x] **REQ-SIM-TRANSFER**: Configuration is realistic enough for sim-to-real transfer.

## Traceability

| Requirement | Phase | Status |
|-------------|-------|--------|
| REQ-ENV-LANE | Phase 1 | COMPLETE |
| REQ-ROBOT-CAM | Phase 2 | COMPLETE |
| REQ-VIS-DETECT | Phase 3 | COMPLETE |
| REQ-CTRL-CENTER | Phase 4 | COMPLETE |
| REQ-NAV-GRAPH | Phase 11 | COMPLETE |
| REQ-NAV-TRANSITION | Phase 12 | COMPLETE |
| REQ-INT-LIGHT | Phase 12 | COMPLETE |
| REQ-INT-CTRL | Phase 12 | COMPLETE |
| REQ-INT-OBS | Phase 12 | COMPLETE |
| REQ-INT-REWARD | Phase 12 | COMPLETE |
| REQ-SIM-METRIC | Phase 10 | COMPLETE |
| REQ-SIM-STABILITY | Phase 10 | COMPLETE |
| REQ-SIM-TRANSFER | Phase 10 | COMPLETE |
| REQ-VIS-HD | Phase 15 | COMPLETE |
| REQ-BACKBONE-ADAPTIVE| Phase 15 | COMPLETE |
| REQ-PROD-HARDENING | Phase 15 | ACTIVE |

---
*Requirements updated: 2026-05-11*
