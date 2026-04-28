# Technology Stack

**Analysis Date:** 2024-04-23

## Languages

**Primary:**
- Python 3.10+ - Core environment, training scripts, and MDP logic. Used throughout `arcproLab/`.

**Secondary:**
- USDA/USD (Universal Scene Description) - Asset definition for robots and environments in `openStreetUSD/` and `arcproLab/assets/`.

## Runtime

**Environment:**
- NVIDIA Isaac Sim 4.0+ / Isaac Lab - Robotics simulation platform and RL framework.

**Package Manager:**
- pip - Standard Python package management.
- Lockfile: `requirements.txt` (present in project root).

## Frameworks

**Core:**
- Isaac Lab (formerly Orbit) - Manager-based RL framework used in `arcproLab/arcpro_env_cfg.py`.
- PyTorch - Deep learning backend for RL policies.

**RL/Training:**
- Stable Baselines 3 (SB3) - RL algorithm implementations.
- SB3-Contrib - Used for `RecurrentPPO` in `arcproLab/scripts/train_policy.py`.

**Build/Dev:**
- NVIDIA Omniverse Kit - Underlying platform for Isaac Sim.

## Key Dependencies

**Critical:**
- `isaaclab` - Core environment and manager abstractions.
- `stable-baselines3` - Policy training and execution.
- `torch` - Tensor computations and neural networks.
- `numpy` - Path processing and math utilities.

**Infrastructure:**
- `omni.usd` - Python API for manipulating USD stages (used in `arcproLab/mdp/track_manager.py`).

## Configuration

**Environment:**
- Configured via `ManagerBasedRLEnvCfg` and `@configclass` decorators.
- Example: `arcproLab/arcpro_env_cfg.py`.

**Build:**
- No explicit build step; Python scripts run directly within Isaac Sim environment.

## Platform Requirements

**Development:**
- Ubuntu 22.04 (standard for Isaac Sim).
- NVIDIA GPU (RTX 3080+ recommended) with latest drivers.

**Production:**
- Isaac Sim Docker or local installation.

---

*Stack analysis: 2024-04-23*
