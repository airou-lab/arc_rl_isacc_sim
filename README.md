# ARCPro RL: Simulation & Environment Stability

This directory contains the reinforcement learning simulation environments, digital twin assets, and training entry points for the ARCPro system.

## Environment: IsaacDirectEnv

The `IsaacDirectEnv` uses the Isaac Sim **Direct API**, bypassing the overhead of ROS2 during training to maximize throughput and physics stability.

### 34-Joint "Stability Pass" (Isaac Sim 2025)
The F1Tenth model is a high-fidelity digital twin with a complex 26-joint suspension and 34 total joints. To prevent physics explosions (NaN errors) in Isaac Sim 2025, the following overrides are applied in `_setup_sim()`:

1.  **Mass Injection:** Rigid body masses (3.0kg chassis, 0.15kg wheels) are explicitly assigned at runtime, overriding potential USD file inconsistencies.
2.  **Damping (50.0) / Stiffness (1000.0):** Applied across all 34 joints via the `ArticulationView` API to dampen high-frequency oscillation in the suspension.
3.  **Advanced Physics Solver:**
    *   `set_position_iteration_count(64)`
    *   `set_velocity_iteration_count(32)`
    *   These settings eliminate mathematical drift and "jittering" at high speeds.

## Vision Stability & Initialization

### Spawn Relocation
The robot is now spawned at `(-125.0, 62.0, 0.5)` with `yaw=0.0`. This specific location ensures the robot starts on a textured, high-contrast section of the track, preventing vision-based policies from "blacking out" during the first few rollout steps.

### Stable Frame Capture
`IsaacDirectEnv` uses the Replicator `AnnotatorRegistry` for frame capture. To ensure stable vision:
-   `simulation_app.update()` is called during `_setup_sim()` to ensure USD stage resources are fully loaded.
-   `_world.step(render=True)` is used for the final physics step before each observation.

## Training Stability: Direct API Singleton

**Critical Requirement:** The `SimulationApp` **must** be initialized before importing any other Isaac Sim or Torch modules in `train_direct.py`. Failure to do so leads to GPU context collisions and "Process Hangs."

```python
# train_direct.py - STABLE INITIALIZATION
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

import torch # Import AFTER SimulationApp
from isaac_direct_env import IsaacDirectEnv # Import AFTER SimulationApp
```
