# External Integrations

**Analysis Date:** 2024-10-24

## APIs & External Services

**Simulation Platform:**
- NVIDIA Isaac Sim / Isaac Lab - The primary simulation engine.
  - SDK/Client: `isaaclab` python package.
  - Auth: No direct authentication required (local installation).

## Data Storage

**Databases:**
- Not detected.

**File Storage:**
- **USD Scene Files:** Integration of Universal Scene Description (USD) for simulation environments.
  - Location: `openStreetUSD/` contains track environments like `no_graph_sim_cleaned.usd`.
- **Robot Assets:**
  - Location: `arcproLab/assets/robot/F1Tenth_Metric.usd`.
- **Model Checkpoints:**
  - Location: `logs/ppo/` for training outputs.
- **Static Map Data:**
  - Location: `arcproLab/mdp/track_centerline.npy` for track logic and reward calculation.

**Caching:**
- None detected.

## Authentication & Identity

**Auth Provider:**
- None (Local system).

## Monitoring & Observability

**Error Tracking:**
- None.

**Logs:**
- **TensorBoard:** SB3 training logs are stored as event files in `logs/ppo/`.
- **Standard Output:** Training progress and simulation diagnostics are logged to console.

## CI/CD & Deployment

**Hosting:**
- Local workstation or HPC servers.

**CI Pipeline:**
- GitHub Actions - Defined in `.github/workflows/ci.yml`. Runs flake8 and pytest.

## Environment Configuration

**Required env vars:**
- `ISAACLAB_PATH` - Path to the Isaac Lab shell script (e.g., `/home/arika/IsaacLab/isaaclab.sh`).
- `LD_LIBRARY_PATH` - Often required by Isaac Sim for finding NVIDIA drivers and libraries.

**Secrets location:**
- Not applicable (no cloud secrets).

## Webhooks & Callbacks

**Incoming:**
- None.

**Outgoing:**
- None.

---

*Integration audit: 2024-10-24*
