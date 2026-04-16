# Technology Stack

**Analysis Date:** 2025-05-15

## Languages

**Primary:**
- Python 3.10+ - RL environment logic, training scripts, and asset management.

**Secondary:**
- Bash - Orchestration scripts (`train.sh`, `verify_sim.sh`, `run_gui_verify.sh`).

## Runtime

**Environment:**
- Isaac Sim 4.0+ - Real-time physics engine and renderer.
- Isaac Lab 1.0+ - Manager-based RL environment orchestration.

**Package Manager:**
- Conda/Mamba - Integrated with Isaac Lab.
- pip - Secondary dependencies.
- Lockfile: missing (managed by base Isaac Lab installation).

## Frameworks

**Core:**
- Isaac Lab 1.0+ - Core orchestration.
- Gymnasium 1.0+ - RL environment interface.

**RL Algorithm:**
- Stable Baselines3 (SB3) - PPO for navigation policy.
- isaaclab_rl - SB3 wrapper for Isaac Lab.

**Build/Dev:**
- PhysX 5.1 - Physics solver (TGS solver type).
- USD (Universal Scene Description) - Industry-standard asset management.

## Key Dependencies

**Critical:**
- `isaaclab.envs.ManagerBasedRLEnv` - Base class.
- `torch` - Backend for NN and vectorized physics.

**Infrastructure:**
- `omni.physx` - Physics interaction (raycasting).
- `numpy` (< 2.0) - Waypoint processing.

## Configuration

**Environment:**
- `arcproLab/arcpro_env_cfg.py` - Main configuration (1x Scale, 500Hz physics, 20Hz control).
- `arcproLab/arcpro_robot_cfg.py` - Robot properties (20kg mass, 1.0x metric scale).

**Build:**
- `train.sh` - Standardized training.
- `run_gui_verify.sh` - GUI-based verification.

## Platform Requirements

**Development:**
- NVIDIA GPU with 8GB+ VRAM (12GB+ for camera rendering).
- Ubuntu 22.04+ with Isaac Lab 1.0+ installed.

**Production:**
- Headless simulation support for training.
- GUI support for verification and visualization.

---

*Stack analysis: 2025-05-15*
