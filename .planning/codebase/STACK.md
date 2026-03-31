# Technology Stack

**Analysis Date:** 2024-10-24

## Languages

**Primary:**
- Python 3.10+ - Core logic, configuration, and RL environment implementation.

**Secondary:**
- USD (Universal Scene Description) - Scene and robot asset representation.
- Bash - Execution scripts (`train.sh`, `verify_sim.sh`).

## Runtime

**Environment:**
- NVIDIA Isaac Lab (formerly Orbit) - Framework for robot learning.
- NVIDIA Isaac Sim - Core simulation engine.
- NVIDIA Omniverse - Underlying platform for USD and simulation.

**Package Manager:**
- Pip - Used for managing Python dependencies.
- Lockfile: missing (uses `requirements.txt`).

## Frameworks

**Core:**
- Isaac Lab - Manager-based RL environment framework.
- PyTorch - Deep learning and GPU-accelerated tensor operations.

**Testing:**
- Pytest - Unit and integration testing.

**Build/Dev:**
- Flake8 - Linting.

## Key Dependencies

**Critical:**
- `isaaclab` - Primary framework for the RL environment.
- `torch` - Neural network training and tensor math.
- `numpy < 2.0` - Data manipulation (restricted version for compatibility).

**Infrastructure:**
- `matplotlib` - Visualization and analytics in `arcproLab/mdp/visual_analytics.py`.
- `pxr` (Pixar USD) - Python bindings for Universal Scene Description.

## Configuration

**Environment:**
- Isaac Lab `configclass` - Decorator-based configuration system used in `arcproLab/arcpro_env_cfg.py`.
- Environment Variables - Used for Isaac Sim paths and CUDA device selection.

**Build:**
- `requirements.txt` - Python dependency list.

## Platform Requirements

**Development:**
- NVIDIA GPU (RTX 30-series or newer recommended).
- NVIDIA Driver (525.60.11+).
- Ubuntu 20.04/22.04.
- Isaac Sim 2023.1.1+.

**Production:**
- NVIDIA Isaac Lab / Isaac Sim environment.

---

*Stack analysis: 2024-10-24*
