# ARCPro RL Environment & Policy Architecture

This directory contains the core IsaacLab environment and training logic for the ARCPro Self-Driving Agent.
For full system architecture, kinematics formulas, and AAC specifications, see the base [README.md](file:///home/arika/Documents/arcpro/arcpro_system/src/examples/ARCPro_RL/arc_rl_isacc_sim/README.md).

---

## 1. How the Self-Driving Policy Works

The agent drives autonomously using an **Asymmetric Actor-Critic (AAC)** paradigm coupled with **inductive kinematic feedforward priors**:

1. **Perception**:
   - Intel RealSense D455 Wide bumper camera (`pos=(0.22, 0.0, 0.18)`, $15^\circ$ downward pitch) streams $224 \times 224$ RGB at 50Hz.
   - Pretrained ResNet-18 conv layers extract $512 \times 7 \times 7$ visual features, compressed by a trainable $1 \times 1$ Conv2D layer (32 channels) and projected into a 256-dim spatial latent vector preserving left (yellow) vs. right (white) road lines.
2. **Actor Control (`ARCProActor`)**:
   - Takes 256 visual features + 12 onboard telemetry features (`v`, `ωz`, `lookahead_κ`, `go_signal`).
   - Privileged spatial error coordinates (lateral error, heading error, distance) are strictly **masked to 0.0**, ensuring 100% camera-only lane following at test time.
   - Computes raw policy action means via MLP (`Linear(128) -> ReLU -> Linear(2)`).
3. **Kinematic Feedforward Priors**:
   - **Steering Prior**: $\text{steer\_bias} = \text{clamp}(0.485 \times \kappa, -1.0, 1.0)$ pulls wheels into turns based on vehicle Ackermann geometry ($L/\delta_{\max}$).
   - **Pre-Braking Throttle Prior**: $\text{brake\_bias} = \text{clamp}(|\kappa| \times 0.35, 0.0, 0.70)$ automatically scales drive action down to $-0.45$ (~11 rad/s, $v \approx 0.35\text{ m/s}$) ahead of sharp $90^\circ$ corners, preventing high-speed understeer.
4. **Privileged Critic (`ARCProCritic`)**:
   - Directly observes unmasked ground-truth track telemetry (lateral error, heading error, lookahead curvature, accumulated distance) to produce low-variance value estimations $V(s)$.

---

## 2. Directory Structure

- [`mdp/`](file:///home/arika/Documents/arcpro/arcpro_system/src/examples/ARCPro_RL/arc_rl_isacc_sim/arcproLab/mdp): Markov Decision Process components:
  - `actions.py`: Ackermann steering and grouped wheel velocity action terms.
  - `observations.py`: Asymmetric telemetry vector assembly (masked for Actor, unmasked for Critic).
  - `rewards.py`: Reward shaping (tanh-bounded progress, curvature speed tracking, heading gate, stationary penalty).
  - `terminations.py`: Roadmark contact and physical track-drop terminations.
  - `events.py`: 5-sector dynamic curriculum spawner with lateral/heading domain randomization.
  - `track_manager.py`: Vectorized spline geometry and lookahead curvature sampling.
  - `curriculum_config.py` & `curriculum_sectors.json`: Dynamic curriculum sector weighting.
- [`agents/`](file:///home/arika/Documents/arcpro/arcpro_system/src/examples/ARCPro_RL/arc_rl_isacc_sim/arcproLab/agents): PPO network definitions:
  - `skrl_models.py`: `ARCProActor` and `ARCProCritic` neural architectures with kinematic feedforward injection.
  - `skrl_wrappers.py`: Environment observation/action adapters for SKRL.
- [`scripts/`](file:///home/arika/Documents/arcpro/arcpro_system/src/examples/ARCPro_RL/arc_rl_isacc_sim/arcproLab/scripts): Execution and operational scripts:
  - `train_skrl.py`: PPO training loop with 3-tier differential Adam learning rates.
  - `autonomous_sentinel.py`: Background daemon monitoring fleet health and executing automated self-healing.
- `assets/` & `models/`: USD track assets and robot meshes.

