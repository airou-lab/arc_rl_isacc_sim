---
wave: 1
---
# Phase 14-01 Plan: Procedural Multi-Agent Scaffolding

This sub-phase establishes the foundation for Intra-Environment Multi-Agent (IEMA) interaction by implementing the Zero-Localization V2V protocol.

## Goals
1. **Procedural Spawning**: Support `num_agents` in the scene configuration with Rough Placement.
2. **FCFS Queue Logic**: Implement the logical V2V queueing based on arrival/exit gates.
3. **Queue-Aware Observations**: Provide Go/Wait and Queue Position signals to the policy.
4. **Global Collision Reset**: Implement inter-agent crash detection.

## Task Breakdown

### Wave 1: Config & Spawning
- [ ] **Task 14-01-01: Create MARL Scaffolding**
    - Implement `for` loops in `ARCProSceneCfg` for N robots.
    - Define staggered spawn points at the 4 branches of the main intersection.
    - Procedurally generate Observation and Action terms for N agents.

### Wave 2: V2V Logic Manager
- [ ] **Task 14-01-02: Implement V2VManager**
    - Create a singleton manager that tracks logical gate triggers.
    - Implement the `ARRIVED`, `STOPPED`, and `CLEARED` event logic.
    - Maintain a First-Come, First-Served (FCFS) queue per environment.

### Wave 3: Integration & Safety
- [ ] **Task 14-01-03: Yielding MDP Terms**
    - Bridge the `V2VManager` status to the telemetry vector.
    - Implement the **Queue-Jumping Penalty (-1000)** and immediate termination logic.
- [ ] **Task 14-01-04: Robot-Robot Collision**
    - Implement `mdp/terminations.py::robot_robot_collision` using `ContactSensor` data.

## Verification Strategy
- **Logic Verification**: Print the `V2VManager` queue status and confirm FCFS order.
- **Visual Verification**: Observe 4 robots in the GUI interacting at the 4-way stop.
- **Safety Verification**: Drive one robot into the intersection out of turn and confirm immediate reset.
