# Technology Stack

**Analysis Date:** 2025-05-21

## Languages

**Primary:**
- Python 3.10+ - All RL logic, environment configuration, and training scripts.

## Runtime

**Environment:**
- NVIDIA Isaac Sim 4.0+ - Primary simulation engine.
- NVIDIA Isaac Lab (formerly Orbit) - Framework for modular RL environments.

**Package Manager:**
- `pip` - Standard Python dependency management.
- Conda/Mamba - Used to manage the Isaac Sim environment.
- Lockfile: `requirements.txt` (present).

## Frameworks

**Core:**
- PyTorch 2.0+ - Neural network backend for RL policies.
- Stable Baselines3 (SB3) - Implementation of PPO for training.

**Simulation:**
- NVIDIA PhysX - Physics engine (configured for TGS solver, CCD enabled).
- OpenUSD - Data format for assets and environment (`.usd`, `.usda`).

**Build/Dev:**
- `AppLauncher` (Isaac Lab) - Manages Isaac Sim process lifecycle.

## Key Dependencies

**Critical:**
- `torch` - Vectorized math and neural networks.
- `isaaclab` - Scene, Observation, Reward, and Action management.
- `numpy` - Data processing for scripts.

**Infrastructure:**
- `omni.usd` / `pxr.Usd` - Low-level manipulation of the simulation stage and marker discovery.
- `PIL` (Pillow) - Image capture and verification.

## Configuration

**Environment:**
- Isaac Sim Python App - Scripts must be launched via the Isaac Sim `python.sh` wrapper.
- Environment variables: `ISAACSIM_PATH`, `OMNI_KIT_PATH`.

**Build:**
- `arcproLab/arcpro_env_cfg.py` - Core simulation and MDP configuration.

## Platform Requirements

**Development:**
- Ubuntu 22.04+ (WSL2 supported).
- NVIDIA GPU (RTX 3080+ recommended) with `nvidia-driver-535+`.
- CUDA 12.1+.

**Production:**
- Headless training on Linux servers with GPU acceleration.

---

*Stack analysis: 2025-05-21*
