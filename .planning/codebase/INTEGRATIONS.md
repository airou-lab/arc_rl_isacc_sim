# External Integrations

**Analysis Date:** 2025-04-28

## APIs & External Services

**Simulation:**
- NVIDIA Isaac Sim 4.0+ - Real-time physics engine and renderer.
  - SDK: `omni`, `isaacsim`.
  - Auth: Local license / NVIDIA NGC.

**RL Framework:**
- Isaac Lab 1.0+ - Environment manager and RL interface.
  - SDK: `isaaclab`.
- Stable Baselines3 (SB3) - Specifically PPO for policy training.
  - SDK: `isaaclab_rl.sb3`.

## Data Storage

**Assets:**
- OpenStreetUSD - USD map tiles and road assets.
  - Primary: `openStreetUSD/no_graph_sim.usd`.
  - Asset Management: Integrated via `UsdFileCfg` in `arcpro_env_cfg.py`.

**Robot Asset:**
- **F1Tenth_Metric.usd**: Metric-calibrated robot asset, scaled 8.0x for simulation stability.
  - Location: `arcproLab/assets/robot/`.
  - Config: `arcproLab/arcpro_robot_cfg.py`.

**Models:**
- PyTorch Weights - Saved as `.pth` or `.zip` files for SB3.
  - Files: `arcproLab/models/road_following_model.pth`.
  - Managed by: SB3 and `verify_policy.py`.

**Navigation Data:**
- Waypoint Arrays - NumPy arrays containing absolute track centerlines.
  - Files: `arcproLab/mdp/track_centerline.npy`.
  - Format: [X, Y, Yaw, Velocity] (Altitude-aware).
  - Managed by: `arcproLab/mdp/track_manager.py`.

## Authentication & Identity

**Auth Provider:**
- None - Local or containerized execution.

## Monitoring & Observability

**Logs:**
- TensorBoard - Real-time training monitoring.
  - Location: `logs/ppo/`.
- Stdout - Used for Phase 09 stabilization verification tools (`verify_spawn.py`, `verify_metric.py`).

## CI/CD & Deployment

**CI Pipeline:**
- GitHub Actions - Automated testing defined in `.github/workflows/ci.yml`.

**Deployment Target:**
- Physical F1Tenth Hardware - Simulation target mimics 20kg mass and 4WD drive.

## Environment Configuration

**Required env vars:**
- None detected; configuration is strictly managed through `@configclass` Python objects.

**Secrets location:**
- Not detected.

## Webhooks & Callbacks

**Incoming:**
- None.

**Outgoing:**
- None.

---

*Integration audit: 2025-04-28*
