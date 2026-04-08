# Technology Stack

**Analysis Date:** 2025-04-28

## Languages

**Primary:**
- Python 3.10+ - Used for all RL environment logic, training scripts, and asset management.

**Secondary:**
- Bash - Used for orchestration scripts (`train.sh`, `verify_sim.sh`, `run_gui_verify.sh`).

## Runtime

**Environment:**
- Isaac Sim 4.0+ (Omniverse)
- Isaac Lab 1.0+ (formerly Orbit)

**Package Manager:**
- Conda/Mamba (Implicitly managed by Isaac Lab environment)
- pip - For secondary dependencies in `requirements.txt`.
- Lockfile: missing (relies on Isaac Lab base container/installation)

## Frameworks

**Core:**
- Isaac Lab 1.0+ - Manager-based environment orchestration.
- Gymnasium 1.0+ - Standard interface for RL environments.

**RL Algorithm:**
- Stable Baselines3 (SB3) - Specifically `PPO` for navigation policy training.
- isaaclab_rl - Isaac Lab wrapper for SB3.

**Build/Dev:**
- PhysX 5.1 - High-fidelity physics solver.
- USD (Universal Scene Description) - Industry-standard asset and scene management.

## Key Dependencies

**Critical:**
- `isaaclab.envs.ManagerBasedRLEnv` - Base class for the environment.
- `torch` - Backend for neural networks and vectorized physics operations.

**Infrastructure:**
- `omni.physx` - Direct interaction with the physics solver for raycasting/snapping logic.
- `numpy` (< 2.0) - Numerical processing for waypoint management.

## Configuration

**Environment:**
- `arcproLab/arcpro_env_cfg.py` - Main simulation and RL configuration.
- `arcproLab/arcpro_robot_cfg.py` - Robot physical properties (Mass: 20kg, Scale: 8.0x).

**Build:**
- `train.sh` - Standardizes training parameters.
- `.github/workflows/ci.yml` - CI configuration.

## Platform Requirements

**Development:**
- NVIDIA GPU with 8GB+ VRAM (12GB+ recommended for cameras).
- Ubuntu 22.04+ with Isaac Lab installed.

**Production:**
- Headless simulation support via `verify_sim.sh` and `train.sh`.

---

*Stack analysis: 2025-04-28*
