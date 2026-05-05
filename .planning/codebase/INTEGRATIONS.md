# External Integrations

**Analysis Date:** 2025-01-24

## APIs & External Services

**Robotics Middleware:**
- ROS2 (Robot Operating System 2) - Used for real-time communication between the RL policy and the robot (simulated or physical).
  - SDK/Client: `rclpy`
  - Topics: `/camera/image_raw`, `/vehicle_state`, `/ackermann_cmd`, `/imu`

**Simulation Platform:**
- NVIDIA Isaac Sim / Isaac Lab - High-fidelity physics simulation and synthetic data generation.
  - SDK: `isaaclab`

## Data Storage

**3D Assets:**
- Universal Scene Description (USD) - Used for robot models and track environments.
  - Location: `arcproLab/assets/` and `openStreetUSD/`

**Trained Models:**
- PyTorch / Stable Baselines 3 Checkpoints - `.zip` and `.pth` files.
  - Location: `arcproLab/models/`

**Calibration & Ground Truth:**
- NPZ files - Used for track boundaries and waypoint data.
  - Example: `arcproLab/mdp/track_boundaries_1x.npz`

## Authentication & Identity

**Auth Provider:**
- Custom / None - Local ROS2 communication and Isaac Sim do not require external auth providers in this setup.

## Monitoring & Observability

**Metrics Visualization:**
- Tensorboard - Used for monitoring RL training progress (loss, rewards, etc.).
  - Implementation: Docker-based setup in `tensorboard_view_docker/`.

**Logging:**
- Python `logging` and ROS2 `node.get_logger()` - Used throughout the codebase for telemetry and debug info.

## CI/CD & Deployment

**Hosting:**
- Local hardware / NVIDIA GPU workstations.

**CI Pipeline:**
- GitHub Actions - Configured in `.github/workflows/ci.yml`.

## Environment Configuration

**Required env vars:**
- `MUJOCO_GL` (optional, for gym renders)
- `DISPLAY` (required for Isaac Sim GUI)
- `PYTHONPATH` (often adjusted to include `arcproLab`)

**Secrets location:**
- Not detected (No external cloud APIs found requiring secrets).

## Webhooks & Callbacks

**Incoming (ROS2 Subscriptions):**
- `/camera/image_raw` - RGB camera feed.
- `/vehicle_state` - Current vehicle telemetry (AckermannDriveStamped).
- `/imu` - IMU sensor data for yaw rate.

**Outgoing (ROS2 Publications):**
- `/ackermann_cmd` - Drive commands (speed, steering, acceleration).

---

*Integration audit: 2025-01-24*
