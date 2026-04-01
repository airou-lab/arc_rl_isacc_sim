# External Integrations

**Analysis Date:** 2024-10-24

## APIs & External Services

**Simulation:**
- NVIDIA Isaac Sim - Real-time physics engine and renderer.
  - SDK: `omni`, `isaacsim`.
  - Auth: Local license / NVIDIA NGC.

**RL Framework:**
- stable-baselines3 / sb3-contrib - Used for RecurrentPPO training.
  - **Telemetry Protocol**: Custom 12-float vector integration.

## Data Storage

**Assets:**
- OpenStreetUSD - USD map tiles and road assets.
  - Files: `openStreetUSD/no_graph_sim_final.usd`.
  - Client: `UsdFileCfg` in `isaaclab.sim`.

**Robot Asset:**
- **F1Tenth_Metric.usd**: 0.5x scale metric robot.
  - Location: `arcproLab/assets/robot/`.

**Models:**
- PyTorch Weights - Saved as `.pth` files.
  - Files: `arcproLab/models/road_following_model.pth`.
  - Library: `torch.load()`.

**Navigation Data:**
- Waypoint Arrays - NumPy arrays containing **absolute** track centerlines.
  - Files: `arcproLab/mdp/track_centerline.npy`.
  - Format: 3D coordinates (altitude-aware).

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
- F1Tenth Hardware - Physical robot using **Front-Wheel Drive (FWD)**.

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

*Integration audit: 2024-10-24*
