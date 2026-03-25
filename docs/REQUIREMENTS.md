# Requirements: ARCPro RL v1.0 (ACHIEVED)

## Functional Requirements
- **FR1 (MET)**: Isaac Sim / Isaac Lab must load the specified USD stage (`no_graph_sim_cleaned.usd`) with the ARCPro robot.
- **FR2 (MET)**: The robot in simulation must be controllable via the Isaac Lab `ActionManager` (Joint Position for steering, Joint Velocity for throttle).
- **FR3 (MET)**: The simulation must provide visual observations via `TiledCamera` and telemetry data via `ObservationManager`.
- **FR4 (MET)**: The RL policy must be loaded from an SB3 model file.
- **FR5 (MET)**: The system must run a closed-loop control where the policy takes observations from the `ARCProEnv` and sends actions back to simulation.

## Technical Requirements
- **TR1 (MET)**: Isaac Lab vectorized environment must be active (`ManagerBasedRLEnv`).
- **TR2 (MET)**: Python environment must have `isaaclab`, `gymnasium`, and `stable_baselines3` installed.
- **TR3 (MET)**: The `ManagerBasedRLEnv` must handle the 160x90 RGB observation space and the 12-element telemetry vector.
- **TR4 (MET)**: Robot articulation (34-joint) must be stable at 1000Hz simulation frequency.
- **TR5 (MET)**: Action Managers in Isaac Lab must map SB3 actions to the robot's steering and drive joints.
