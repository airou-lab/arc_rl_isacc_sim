# Phase 14: Multi-Agent Environment Refactor - Research

**Updated:** 2026-05-06
**Domain:** Zero-Localization V2V & MARL Coordination
**Confidence:** HIGH

## Summary
The project is prototyping a **Zero-Localization V2V (Vehicle-to-Vehicle)** coordination system. The goal is to enable 3-4 robots to navigate a shared 4-way stop intersection without requiring GPS or SLAM. Coordination is achieved through a **Peer-to-Peer Queueing** protocol based on logical "Lane Gates."

## Zero-Localization V2V Protocol

### 1. The "Lane Gate" as a Logical Sensor
The infrastructure (or the robots' local perception) identifies three critical zones per intersection branch:
- **Arrival Gate:** Triggered when a robot approaches the intersection stop line.
- **Stop Detection:** Triggered when the robot's local velocity $v < 0.1 m/s$ at the stop line.
- **Exit Gate:** Triggered when the robot successfully clears the opposite side of the intersection.

### 2. Peer-to-Peer Queueing Logic (FCFS)
Robots broadcast short-range status messages (BSM-Lite):
- **Event: ARRIVED**: "I am at Branch [North/South/East/West] at Timestamp [T]."
- **Event: STOPPED**: "I am stationary at the stop line."
- **Event: CLEARED**: "I have passed the Exit Gate."

**Arbitration Rule:** 
A robot may only enter the intersection if it is at the head of the global **First-Come, First-Served (FCFS)** queue. It yields to any robot that broadcasted a "STOPPED" event with a timestamp earlier than its own.

## Architecture Refinement (V2V Prototype)

### 1. Spawning & "Rough Placement" Strategy
- **Procedural Loop:** Use a `for i in range(num_agents)` loop within the `ARCProSceneCfg` to spawn robots.
- **Rough Placement Strategy:** Agents are spawned at the four entry branches (N, S, E, W) of the main intersection to maximize interaction density.
- **Proximity Maximization:** This ensures agents are within V2X communication range and on potential collision courses immediately.

### 2. Observation Space (The "Peer Table")
The policy does not receive world-coordinates ($x, y$). Instead, it sees:
- **Go/Wait Signal (1 bit):** 1.0 if the robot is at the head of the FCFS queue, 0.0 otherwise.
- **Queue Position (Float):** Current rank in the queue (e.g., 0.0 = Next, 1.0 = Second in line).
- **Branch Occupancy (4 bits):** Binary indicators for North, South, East, and West branches showing if a peer is currently "Committing" to the intersection.

### 3. Reward & Termination (The Social Contract)
- **Queue-Jumping Penalty:** Massive negative reward (-1000) and **immediate termination** if a robot moves into the intersection area while the Go Signal is 0.0.
- **Collision Policy (Global Reset):** Any inter-agent contact results in a reset of the entire environment instance.
- **Success Bonus:** Reward for clearing the Exit Gate after a valid "Go" sequence.

## Technical Decisions
- **RL Framework:** `skrl` with Independent PPO (`IPPO`).
- **Simulation Scaling:** Prototype with `num_envs=1` for visual fidelity, scaling to 32+ envs for production.
- **Environment Base:** `ManagerBasedRLEnv` refactored for procedural N-agent support.

## Implementation Path (Refined)
1. **Wave 1: MARL Config**: Procedural 4-agent spawning and basic MDP term loops.
2. **Wave 2: V2V Manager**: Logic for tracking logical gates and maintaining the FCFS queue.
3. **Wave 3: Yielding Observations**: Bridging the queue status to the robot telemetry.
