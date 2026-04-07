# Technology Stack

**Analysis Date:** 2025-04-18

## Languages

**Primary:**
- Python 3.11/3.12 - Core logic, simulation configuration, and training scripts.

## Runtime

**Environment:**
- NVIDIA Isaac Sim 4.0.0+ - Primary simulation engine.
- NVIDIA IsaacLab 1.0.0+ - Framework for manager-based RL environments.
- Gymnasium 1.0.0+ - Standard RL interface.

**Package Manager:**
- pip - Managed via `requirements.txt`.
- Virtual Environment: `venv/` (Python 3.11/3.12) observed.
- Lockfile: missing

## Frameworks

**Core:**
- IsaacLab - Environment and scene management (`isaaclab.envs`, `isaaclab.managers`).
- Stable Baselines3 (SB3) - RL algorithms and policy structures.
- SB3-contrib - Recurrent PPO support for hierarchical policies.

**Testing:**
- pytest - Unit and integration testing.

**Build/Dev:**
- USD (Universal Scene Description) - Asset format for scenes and robots.
- **Robot Scale**: 8.0x metric scale override in `arcproLab/arcpro_env_cfg.py` for visual/physics stability matching.
- **Track Scale**: 1.0x metric scale (original) in `openStreetUSD/no_graph_sim.usd`.

## Key Dependencies

**Critical:**
- `torch` (PyTorch) - Neural network backend for policies.
- `numpy` (<2.0) - Numerical computations and absolute waypoint handling.
- `sb3_contrib` - Required for `RecurrentPPO` and `RecurrentActorCriticPolicy`.
- `isaaclab_rl.sb3` - Fixed import path for `Sb3VecEnvWrapper`.

**Infrastructure:**
- `matplotlib` - Used for visual analytics and metric verification.
- `isaaclab` - Provides the `ManagerBasedRLEnv` abstraction.

## Configuration

**Environment:**
- Configured via Python classes using `@configclass` decorator from `isaaclab.utils`.
- **Telemetry Protocol**: Standardized 12-float vector mapping in `arcproLab/mdp/observations.py`.
- Key configs: `arcproLab/arcpro_env_cfg.py` (Scene/MDP) and `arcproLab/arcpro_robot_cfg.py` (Robot).

**Physics Optimizations (Phase 08 - Baseline):**
- **Simulation Frequency**: 200Hz (`dt=0.005`).
- **Render Frequency**: 25Hz (`render_interval=8`, `decimation=8`).
- **Solver**: PhysX TGS (`solver_type=1`).
- **Accuracy**: 8 position / 4 velocity iterations.
- **Stability**: `enable_ccd=True`, `enable_stabilization=True` for high-speed AWD (Four-Wheel Drive) motion.

## Platform Requirements

**Development:**
- Ubuntu 22.04+ (Standard for Isaac Sim).
- NVIDIA GPU with RTX support (Required for Isaac Sim).
- NVIDIA Driver 535+ (Recommended).

**Production:**
- Deployment to F1Tenth hardware (sim-to-real) at 1.0x metric scale.
- **4WD (Four-Wheel Drive)** actuator configuration.
- **20kg Mass** constant for realistic vehicle dynamics.

---

*Stack analysis: 2025-04-18*
