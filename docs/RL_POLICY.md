# RL Policy Architecture (Legacy)

> **STATUS:** PORTING TO ISAAC LAB. Custom policy logic is being moved from the Direct API wrappers to the Isaac Lab `ManagerBasedRLEnv`.

## Core Logic to Port
1.  **Hierarchical HPPO:** Port the LSTM-based planning and control heads to the Isaac Lab `ObservationManager`.
2.  **Telemetry Vector:** Port the 12-element ROS2-compatible state vector (indices 0-11).
3.  **Vision Pipeline:** Adapt the 160x90 downsampling for Isaac Lab's `TiledCamera`.
