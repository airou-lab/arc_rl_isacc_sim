# Phase 13 Validation: Live Policy GUI

## Acceptance Criteria
- [ ] **REQ-GUI-FPS (Throughput):** The sidecar GUI maintains a refresh rate > 30 FPS for RGB video without dropping simulation FPS below 100.
- [ ] **REQ-GUI-LAT (Latency):** Total end-to-end latency from simulation camera update to GUI display is < 20ms.
- [ ] **REQ-GUI-RES (Robustness):** The GUI process correctly unlinks shared memory segments on exit (no "Zombie SHM").
- [ ] **REQ-GUI-VIS (Visualization):** Telemetry vectors and PPO action distributions (mean/std) are rendered clearly.

## Verification Methods
- **Performance Test:** `python3 arcproLab/scripts/verify_gui_performance.py` (Measure FPS and Jitter).
- **Integration Test:** Run `train_policy.py` and manually verify that `policy_dashboard.py` displays live video and telemetry.
