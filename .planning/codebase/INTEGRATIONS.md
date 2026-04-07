# External Integrations

**Analysis Date:** 2025-04-18

## APIs & External Services

**Simulation:**
- NVIDIA Isaac Sim - Real-time physics engine and renderer.
  - SDK: `omni`, `isaacsim`.
  - Auth: Local license / NVIDIA NGC.

**RL Framework:**
- stable-baselines3 / sb3-contrib - Used for RecurrentPPO training.
  - **Telemetry Protocol**: Custom 12-float vector integration in `arcproLab/mdp/observations.py`.

## Data Storage

**Assets:**
- OpenStreetUSD - USD map tiles and road assets.
  - Files: `openStreetUSD/no_graph_sim.usd` (Original scale 1.0x).
  - Client: `UsdFileCfg` in `isaaclab.sim` (referenced in `arcproLab/arcpro_env_cfg.py`).

**Robot Asset:**
- **F1Tenth_Metric.usd**: Metric robot asset, scaled 8.0x for the environment.
  - Location: `arcproLab/assets/robot/`.
  - Config: `arcproLab/arcpro_robot_cfg.py`.

**Models:**
- PyTorch Weights - Saved as `.pth` files.
  - Files: `arcproLab/models/road_following_model.pth`.
  - Library: `torch.load()`.

**Navigation Data:**
- Waypoint Arrays - NumPy arrays containing **absolute** track centerlines.
  - Files: `arcproLab/mdp/track_centerline.npy`.
  - Format: 3D coordinates (altitude-aware).
  - Managed by: `arcproLab/mdp/track_manager.py`.

## Authentication & Identity

**Auth Provider:**
- None - Local execution.

## Monitoring & Observability

**Logs:**
- Local TensorBoard - Standard for SB3 training.
  - Location: `logs/ppo/`.

## CI/CD & Deployment

**CI Pipeline:**
- GitHub Actions - Defined in `.github/workflows/ci.yml`.

**Deployment Target:**
- F1Tenth Hardware - Physical robot using **4WD (Four-Wheel Drive)** and **20kg** mass.

## Environment Configuration

**Required env vars:**
- No explicit env vars in current code; configuration is strictly through Python `configclass`.

**Secrets location:**
- Not detected / None.

## Webhooks & Callbacks

**Incoming:**
- None.

**Outgoing:**
- None.

---

*Integration audit: 2025-04-18*
