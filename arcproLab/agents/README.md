# Agents

Contains the SKRL-based Asymmetric Actor-Critic reinforcement learning definitions:

- `skrl_models.py`: Equivariant ResNet-18 + `SpatialSoftmax2d` `ARCProActor` (actor-masked telemetry) and privileged `ARCProCritic` (`asinh` odometer compression + running normalization).
- `skrl_wrappers.py`: IsaacLab-to-SKRL wrapper with 2-slot ping-pong observation ring buffer (`s_t` / `s_{t+1}` non-aliasing) and direct static camera reads.
- `telemetry_ppo.py`: `TelemetryPPO` trainer with decoupled Actor/Critic optimizers, symmetry regularization (`L_sym`), and TensorBoard telemetry logging.
