# Phase 2 Research: Policy Loading and Inference

## 1. Model Architecture
- **Algorithm:** `RecurrentPPO` from `sb3_contrib`.
- **Policy Class:** `HierarchicalPathPlanningPolicy` (Custom).
- **Feature Extractor:** `FusionFeaturesExtractor` (Custom).
- **Environment:** Compatible with `isaac_direct_env.py` (Gymnasium-based).

## 2. Loading Requirements
To successfully load `final_model.zip`, the following must be in the Python `sys.path`:
- `/home/arika/Documents/arcpro/arcpro_system/src/examples/ARCPro_RL/arc_rl_isacc_policy`
- This ensures the `policies` module is importable as required by the `cloudpickle` deserializer in SB3.

## 3. Observation Mapping (Confirmed)
- **Image:** 160x90 RGB.
- **Vector (12 elements):**
    - [0] turn_bias
    - [1] reserved
    - [2] goal_dist_masked
    - [3] speed
    - [4] yaw_rate
    - [5] last_steer
    - [6] last_throttle
    - [7] last_brake
    - [8] lat_err_zero
    - [9] hdg_err_zero
    - [10] kappa_zero
    - [11] total_dist

## 4. Inference Performance
- **RNN State:** `RecurrentPPO` requires passing and updating the `lstm_states` between inference steps.
- **Latency:** Isaac Sim 2025's `SimulationApp` can handle the Torch inference loop synchronously without significant frame drops at 20Hz.
