# External Integrations

**Analysis Date:** 2026-07-28

## APIs & External Services

**Robotics & Simulation:**
- ROS 2 Middleware - Underlying messaging fabric (Publishers, Subscribers, Actions)
  - SDK/Client: `rclpy`, `std_msgs`, `sensor_msgs`
  - Auth: None
- NVIDIA Isaac Sim - Physical simulation engine
  - SDK/Client: Omniverse USD API (`pxr`), `isaac_ros2_env`
  - Auth: None

**Machine Learning:**
- TensorBoard - Visualizing training metrics and performance
  - SDK/Client: `tensorboard`
  - Auth: None (local)

## Data Storage

**Databases:**
- SQLite3 (via `ros2bag-sqlite3-cli` 0.26.9)
  - Connection: Local file
  - Client: ROS 2 Bag

**File Storage:**
- Local filesystem only (for saving models, checkpoints, and logs in `logs/` and `archive/`)

**Caching:**
- None

## Authentication & Identity

**Auth Provider:**
- None (Local robotics application)
  - Implementation: Custom/Local only

## Monitoring & Observability

**Error Tracking:**
- None

**Logs:**
- Local text files (`crash_test.log`, `debug_vision.log`, `output.log`)
- ROS 2 Logging (`rcutils`, `logging-demo`)
- TensorBoard (`tensorboard` tracking)

## CI/CD & Deployment

**Hosting:**
- Local workstation execution or Docker containers (e.g., `tensorboard_view_docker/`)

**CI Pipeline:**
- GitHub Actions (indicated by presence of `.github/` directory)

## Environment Configuration

**Required env vars:**
- Local execution relies on ROS 2 setup (`ROS_DOMAIN_ID`, `setup.bash`) and Omniverse paths.

**Secrets location:**
- Not applicable for this local robotics simulation

## Webhooks & Callbacks

**Incoming:**
- None

**Outgoing:**
- None

---

*Integration audit: 2026-07-28*
