# Technology Stack

**Analysis Date:** 2026-07-28

## Languages

**Primary:**
- Python 3.12.3 - Core simulator control, reinforcement learning logic, and agent scripts.
- JavaScript / HTML / CSS - For the `arcpro-rl-course` frontend content.

**Secondary:**
- C/C++ - Via dependencies like pybind11, CUDA, and ROS 2 bindings.

## Runtime

**Environment:**
- Python 3.12.3 (virtual environment)
- ROS 2 (likely Humble or Iron based on package versions)

**Package Manager:**
- pip
- Lockfile: missing (relies on `archive/requirements.txt`)

## Frameworks

**Core:**
- Gymnasium 1.2.3 - RL environment interface
- Stable Baselines 3 (`stable_baselines3`, `sb3_contrib`) 2.7.1 - Primary RL algorithms (e.g., PPO)
- skrl - Alternative reinforcement learning framework
- ROS 2 (`rclpy` 7.1.9) - Middleware for robotics messaging and component architecture

**Testing:**
- pytest 9.0.2 - Test framework for unit/integration tests

**Build/Dev:**
- ament - ROS 2 build system and linting (ament-flake8, ament-pep257, ament-lint)
- setuptools 82.0.1 - Package building

## Key Dependencies

**Critical:**
- torch 2.10.0 - Deep learning backbone
- torchvision 0.25.0 - Vision models and transforms
- numpy 2.4.3 - Mathematical computing and array manipulation
- pxr (USD) - Pixar's Universal Scene Description API for Isaac Sim integration (`archive/inspect_usd.py`)

**Infrastructure:**
- tensorboard 2.20.0 - Training logging and visualization
- opencv-python-headless 4.13.0.92 - Image processing
- nav2 (`nav2-msgs`, `nav2-common`) - ROS 2 navigation stack integrations
- NVIDIA packages (`nvidia-cudnn-cu12`, `cuda-bindings`) - GPU compute infrastructure

## Configuration

**Environment:**
- Configured via Python scripts (e.g., `arcproLab/arcpro_env_cfg.py`, `arcproLab/arcpro_robot_cfg.py`)
- `.env` files not detected, relies on explicit python configs.

**Build:**
- ROS 2 patterns for building (ament) and standard python setup.

## Platform Requirements

**Development:**
- NVIDIA GPU with CUDA 12.8 support (required for torch, cuda-bindings, nvidia-* packages)
- Linux (ROS 2 + Isaac Sim typically requires Ubuntu)

**Production:**
- NVIDIA Omniverse / Isaac Sim environment
- Local workstation or Docker container (e.g., `tensorboard_view_docker/`)

---

*Stack analysis: 2026-07-28*
