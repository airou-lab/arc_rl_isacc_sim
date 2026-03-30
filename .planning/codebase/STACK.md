# Technology Stack

**Analysis Date:** 2024-10-24

## Languages

**Primary:**
- Python 3.10+ - Core logic, simulation scripting, and reinforcement learning.

**Secondary:**
- Bash - Automation scripts for training and verification (`train.sh`, `verify_sim.sh`).
- C++ - Underlying engine for NVIDIA Isaac Sim and USD (not directly edited).

## Runtime

**Environment:**
- NVIDIA Isaac Sim / Isaac Lab - High-fidelity robotics simulation environment.
- CUDA - GPU acceleration for simulation and deep learning.

**Package Manager:**
- pip - Managed via `requirements.txt`.
- Lockfile: missing (no `requirements.lock` or `poetry.lock`).

## Frameworks

**Core:**
- Isaac Lab - Robot learning framework built on NVIDIA Isaac Sim.
- PyTorch 2.x - Deep learning framework for policy training and inference.
- Stable Baselines3 (SB3) - Implementation of PPO and other RL algorithms.
- Gymnasium - Standard API for reinforcement learning environments.

**Testing:**
- pytest - Unit testing framework for logic like track management.

**Build/Dev:**
- NVIDIA Omniverse - Underlying platform for Isaac Sim.
- Pixar USD (Universal Scene Description) - Scene and robot description format.

## Key Dependencies

**Critical:**
- `isaaclab` - Provides the `ManagerBasedRLEnv` and simulation utilities.
- `torch` - Tensor computations and neural network modules.
- `stable-baselines3` - RL training loop and algorithm implementations.
- `numpy` (<2.0) - Numerical computations, specifically for track data.

**Infrastructure:**
- `pxr-usd` (Pixar USD) - Low-level USD API for scene manipulation.
- `omni.isaac.*` - Isaac Sim core extensions.

## Configuration

**Environment:**
- Configured via Python classes using `isaaclab.utils.configclass`.
- Environment variables like `ISAACLAB_PATH` are used to locate the simulator.

**Build:**
- No traditional build step; Python scripts are executed via the `isaaclab.sh` wrapper.

## Platform Requirements

**Development:**
- Ubuntu 20.04/22.04 (standard for Isaac Sim).
- NVIDIA GPU (RTX 30-series or higher recommended).
- NVIDIA Driver 525+.

**Production:**
- Headless Linux servers with high-performance NVIDIA GPUs.

---

*Stack analysis: 2024-10-24*
