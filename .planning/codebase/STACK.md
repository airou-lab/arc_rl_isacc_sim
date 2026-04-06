# Technology Stack

**Analysis Date:** 2025-04-04

## Languages

**Primary:**
- Python 3.11/3.12 - Core logic, simulation configuration, and training scripts.

## Runtime

**Environment:**
- NVIDIA Isaac Sim 4.0.0+ - Primary simulation engine.
- NVIDIA IsaacLab - Framework for manager-based RL environments.

**Package Manager:**
- pip - Managed via `requirements.txt`.
- Virtual Environment: `.venv/` (Python 3.12) and `venv/` (Python 3.11) observed.

## Frameworks

**Core:**
- IsaacLab - Environment and scene management (`isaaclab.envs`, `isaaclab.managers`).
- Stable Baselines3 (SB3) - RL algorithms and policy structures.
- SB3-contrib - Recurrent PPO support for hierarchical policies.

**Testing:**
- pytest - Unit and integration testing.

**Build/Dev:**
- USD (Universal Scene Description) - Asset format for scenes and robots.
- **Robot Scale**: 1.0x metric scale (canonical) for F1Tenth assets. Note: `arcpro_env_cfg.py` currently uses an 8.0x override for scene matching.

## Key Dependencies

**Critical:**
- `torch` (PyTorch) - Neural network backend for policies.
- `numpy` (<2.0) - Numerical computations and absolute waypoint handling.
- `sb3_contrib` - Required for `RecurrentPPO` and `RecurrentActorCriticPolicy`.

**Infrastructure:**
- `matplotlib` - Used for visual analytics and metric verification.
- `isaaclab` - Provides the `ManagerBasedRLEnv` abstraction.

## Configuration

**Environment:**
- Configured via Python classes using `@configclass` decorator from `isaaclab.utils`.
- **Telemetry Protocol**: Standardized 12-float vector mapping in `mdp/observations.py`.
- Key configs: `arcproLab/arcpro_env_cfg.py` (Scene/MDP) and `arcproLab/arcpro_robot_cfg.py` (Robot).

**Build:**
- No formal build system; relies on Python scripts and USD asset loading.

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

*Stack analysis: 2025-04-04*
