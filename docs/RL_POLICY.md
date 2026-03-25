# RL Policy Architecture (v1.0)

> **STATUS:** PORTED TO ISAAC LAB. Custom policy logic has been successfully moved from the Direct API wrappers to the Isaac Lab `ManagerBasedRLEnv`.

## Ported Components
1.  **Hierarchical HPPO:** Ported the LSTM-based planning and control heads to the Isaac Lab `ObservationManager`.
2.  **Telemetry Vector:** Successfully implemented the 12-element state vector in `mdp/observations.py`.
3.  **Vision Pipeline:** Successfully adapted the 160x90 downsampling for Isaac Lab's `TiledCamera`.
