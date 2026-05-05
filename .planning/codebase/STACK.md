# Technology Stack

**Analysis Date:** 2025-01-24

## Languages

**Primary:**
- Python 3.10+ - Core logic for RL training, environment configuration, and ROS2 deployment.

**Secondary:**
- Universal Scene Description (USD/USDA) - 3D scene representation, asset modeling, and physics configuration for NVIDIA Isaac Sim.

## Runtime

**Environment:**
- NVIDIA Isaac Sim / Isaac Lab - Primary simulation and physics environment.
- ROS2 (Robot Operating System) - Distributed communication middleware for deployment and hardware integration.
- CUDA - GPU acceleration for both simulation physics (PhysX) and neural network computations.

**Package Manager:**
- pip - Python package management.
- Lockfile: missing (using `requirements.txt`).

## Frameworks

**Core:**
- NVIDIA Isaac Lab - High-level framework for Reinforcement Learning on Isaac Sim.
- PyTorch - Deep learning backend for RL policies and inference.
- ROS2 (rclpy) - Communication layer for the `IsaacROS2Env` deployment environment.

**Testing:**
- pytest - Testing framework used in `arcproLab/policy_stack/pytest.ini`.

**Build/Dev:**
- Docker - Used for containerized services like Tensorboard (`tensorboard_view_docker/`).

## Key Dependencies

**Critical:**
- `torch` - Neural network training and tensor operations.
- `isaaclab` - RL environment management and Isaac Sim utilities.
- `stable-baselines3` / `sb3_contrib` - RL algorithms, specifically `RecurrentPPO` for sequential decision making.
- `gymnasium` - Standard API for Reinforcement Learning environments.

**Infrastructure:**
- `rclpy` - ROS2 Python client library.
- `numpy` - Numerical data processing for observations and telemetry.
- `opencv-python` (cv2) - Computer vision for lane detection and image resizing.
- `cv_bridge` - Bridge between ROS2 image messages and OpenCV.

## Configuration

**Environment:**
- Configured via Python `@configclass` decorators in `arcproLab/arcpro_env_cfg.py` and `arcproLab/arcpro_robot_cfg.py`.
- ROS2 topics and parameters configured via `IsaacROS2Config` dataclass in `arcproLab/policy_stack/isaac_ros2_env.py`.

**Build:**
- GitHub Actions workflows in `.github/workflows/ci.yml`.

## Platform Requirements

**Development:**
- NVIDIA GPU with CUDA support.
- Ubuntu 20.04/22.04 (standard for Isaac Sim and ROS2).
- Isaac Sim installation.

**Production:**
- ARCPro / F1TENTH hardware or Isaac Sim for high-fidelity verification.

---

*Stack analysis: 2025-01-24*
