# External Integrations

**Analysis Date:** 2024-04-23

## APIs & External Services

**Simulation:**
- NVIDIA Isaac Sim - Primary simulation engine for physics and visuals.
  - SDK: `isaaclab`, `omni.isaac.*`
  - Integration: `arcproLab/arcpro_env_cfg.py`

**RL Algorithms:**
- Stable Baselines 3 - Provides PPO and RecurrentPPO algorithms.
  - Integration: `arcproLab/scripts/train_policy.py`

## Data Storage

**Assets:**
- USD (Universal Scene Description) - Local files containing 3D meshes and physics properties.
  - Track: `openStreetUSD/no_graph_sim_clean_1x.usda`
  - Robot: `arcproLab/assets/robot/F1Tenth_Metric.usd`

**Weights & Logs:**
- Tensorboard - Used for monitoring training progress.
  - Logs location: `logs/sb3/` (generated during training)
- PyTorch Models - Stored as `.zip` (SB3 format) or `.pth` (raw weights).
  - Example: `arcproLab/models/road_following_model.pth`

## Authentication & Identity

**Auth Provider:**
- None / Local - The system runs entirely in a local or containerized environment without external auth requirements.

## Monitoring & Observability

**Error Tracking:**
- None - Standard Python traceback and simulation console logs.

**Logs:**
- Console Output - Detailed logs from Isaac Lab and SB3.
- Tensorboard - Visualizes reward curves and environment metrics.

## CI/CD & Deployment

**Hosting:**
- Local workstations or GPU-enabled servers.

**CI Pipeline:**
- GitHub Actions - `.github/workflows/ci.yml` present for code linting/testing.

## Environment Configuration

**Required env vars:**
- `ISAAC_SIM_PATH` - Path to the Isaac Sim installation.
- `DISPLAY` - Required for GUI mode.

**Secrets location:**
- Not applicable - No external API keys used.

## Webhooks & Callbacks

**Incoming:**
- None

**Outgoing:**
- None

---

*Integration audit: 2024-04-23*
