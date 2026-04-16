# External Integrations

**Analysis Date:** 2025-05-15

## APIs & External Services

**Simulation:**
- NVIDIA Isaac Sim 4.0+ - Real-time physics engine and renderer.
  - SDK: `omni`, `isaacsim`.

**RL Framework:**
- Isaac Lab 1.0+ - Environment manager and RL interface.
  - SDK: `isaaclab`.
- Stable Baselines3 (SB3) - PPO for policy training.
  - SDK: `isaaclab_rl.sb3`.

## Data Storage

**Assets:**
- OpenStreetUSD - USD map tiles and road assets.
  - Primary: `openStreetUSD/no_graph_sim_clean_1x.usda` (Clean 1x metric version).
  - Asset Management: Integrated via `UsdFileCfg` with `scale=(0.125, 0.125, 0.125)`.

**Robot Asset:**
- **F1Tenth_Metric.usd**: 1.0x metric-calibrated robot asset.
  - Location: `arcproLab/assets/robot/`.
  - Config: `arcproLab/arcpro_robot_cfg.py`.

**Models:**
- PyTorch Weights - Saved as `.pth` or `.zip` files for SB3.
  - Primary Model: `arcproLab/models/road_following_model.pth`.
  - Checkpoints: `logs/ppo/`.

**Navigation Data:**
- Waypoint Arrays - NumPy arrays containing absolute track centerlines.
  - Files: `arcproLab/mdp/track_centerline_1x.npy` (1x metric scaled).
  - Format: [X, Y, Yaw, Velocity] (Altitude-aware).
  - Managed by: `arcproLab/mdp/track_manager.py`.

## Authentication & Identity

**Auth Provider:**
- None - Local or containerized execution.

## Monitoring & Observability

**Logs:**
- TensorBoard - Real-time training monitoring in `logs/ppo/`.
- Stdout - Real-time termination logs and telemetry audits.
- Telemetry UI: `mdp/visual_analytics.py` provides real-time graphing of speed, lateral error, and joint velocities.

## CI/CD & Deployment

**CI Pipeline:**
- GitHub Actions - Automated testing in `.github/workflows/ci.yml`.

**Deployment Target:**
- Physical F1Tenth Hardware - Simulation target mimics 20kg mass and 4WD drive at 1.0x metric scale.

## Environment Configuration

**Required env vars:**
- None detected; managed through `@configclass` Python objects.

## Webhooks & Callbacks

**Incoming:**
- None.

**Outgoing:**
- None.

---

*Integration audit: 2025-05-15*
