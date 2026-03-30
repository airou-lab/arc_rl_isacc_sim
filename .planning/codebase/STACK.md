# Technology Stack

**Analysis Date:** 2025-03-21

## Languages

**Primary:**
- Python 3.12 - Core logic, training scripts, environment configuration, and inference.

**Secondary:**
- USD (Universal Scene Description) - Asset definition for robots (`arcproLab/assets/robot/F1Tenth_Metric.usd`) and environments (`openStreetUSD/no_graph_sim_cleaned.usd`).
- Bash - Execution scripts for training and verification (`train.sh`, `run_gui_verify.sh`).

## Runtime

**Environment:**
- NVIDIA Omniverse Isaac Sim - Underlying simulation engine.
- Isaac Lab v1.0.1 - High-level framework for RL in Isaac Sim.
- Ubuntu 22.04 - Recommended host OS for NVIDIA Omniverse.

**Package Manager:**
- `pip` - Used for managing Python dependencies.
- Lockfile: Missing (relying on `requirements.txt` and environment setup).

## Frameworks

**Core:**
- Isaac Lab v1.0.1 - Used for environment management (`ManagerBasedRLEnv`) and scene configuration.
- Stable Baselines3 (SB3) - Used for training high-level PPO policies (`arcproLab/scripts/train_policy.py`).
- PyTorch (torch) - Backend for both SB3 and the vision-based ResNet18 model (`arcproLab/mdp/policy_wrapper.py`).

**Testing:**
- pytest - Unit testing framework for core logic (`tests/test_track_manager.py`).

**Build/Dev:**
- flake8 - Python linting tool (used in CI).
- GitHub Actions - CI pipeline for linting and unit tests (`.github/workflows/ci.yml`).

## Key Dependencies

**Critical:**
- `isaaclab` - Essential for all simulation-based logic.
- `stable-baselines3` - Provides the PPO implementation for high-level control.
- `torch` - Deep learning framework for inference and training.
- `torchvision` - Used for ResNet18 model architecture and image transforms.

**Infrastructure:**
- `numpy < 2` - Numerical operations (constrained to < 2.0 for compatibility with Omniverse/Isaac Lab).
- `matplotlib` - Data visualization for analytics and debugging.

## Configuration

**Environment:**
- Managed via Python-based configuration classes using `@configclass` from `isaaclab.utils`.
- Main config: `arcproLab/arcpro_env_cfg.py` for the simulation environment.
- Robot config: `arcproLab/arcpro_robot_cfg.py` for the F1Tenth vehicle.

**Build:**
- No standard build system (e.g., setuptools/poetry) detected. Uses absolute and relative path manipulation in scripts.

## Platform Requirements

**Development:**
- NVIDIA GPU with RTX support (required for Isaac Sim).
- NVIDIA Drivers compatible with CUDA 12.x.
- Python 3.12.

**Production:**
- Deployment target: Local workstation or server with NVIDIA GPU for real-time simulation.

---

*Stack analysis: 2025-03-21*
