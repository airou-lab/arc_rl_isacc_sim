# Technology Stack

**Analysis Date:** 2026-05-11

## Languages

**Primary:**
- Python 3.12 - Used for environment configuration, RL logic, and simulation scripts.

**Secondary:**
- USD (Universal Scene Description) - Used for simulation assets and scene definitions in `openStreetUSD/`.

## Runtime

**Environment:**
- NVIDIA Isaac Sim / Isaac Lab - Robotics simulation platform.
- CUDA - Used for GPU-accelerated simulation and neural network training.

**Package Manager:**
- pip - Standard Python package manager.
- Lockfile: missing (using `requirements.txt`).

## Frameworks

**Core:**
- Isaac Lab (formerly Orbit) - Framework for reinforcement learning on Isaac Sim.
- PyTorch - Deep learning framework used for policies and observations.

**Testing:**
- pytest - Used for unit and integration testing in `arcproLab/policy_stack/tests/`.

**Build/Dev:**
- Docker - Used for Tensorboard visualization in `tensorboard_view_docker/`.

## Key Dependencies

**Critical:**
- `isaaclab` - Core simulation and RL environment framework.
- `torch` - Neural network backend.
- `numpy` - Numerical computations for waypoints and track management.
- `sb3_contrib` - Recurrent PPO implementation and supporting types (`RNNStates`).

**Infrastructure:**
- `stable-baselines3` (SB3) - RL algorithm implementations.

## Configuration

**Environment:**
- Configured via Python classes using `isaaclab.utils.configclass`.
- Key configs: `arcproLab/arcpro_env_cfg.py` and `arcproLab/arcpro_robot_cfg.py`.

**MDP Logic (Mastery Calibration):**
- **Hybrid Boundary Logic:** Implemented via `mdp/rewards.py` and `mdp/terminations.py`.
  - Penalty Zone: 0.15m (Weight 1.0, -100.0 penalty).
  - Hard-Reset Zone: 0.25m (Termination threshold).
  - Margin: 10cm survival buffer between penalty and reset.

**Build:**
- `pytest.ini` for testing configuration in `arcproLab/policy_stack/`.

## Platform Requirements

**Development:**
- Ubuntu 22.04+ (required for Isaac Sim).
- NVIDIA GPU with RTX support (RTX 3090/4090 recommended for MARL).
- NVIDIA Driver 525+.

**Production:**
- Deployment to physical ARCPro robots using compatible RL deployment stacks (F1Tenth-based).

---

*Stack analysis: 2026-05-11*
