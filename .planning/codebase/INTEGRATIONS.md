# External Integrations

**Analysis Date:** 2026-05-16

## APIs & External Services

**Simulation:**
- NVIDIA Isaac Sim - Primary simulation engine for physics and rendering.
  - SDK: `isaaclab`, `omni`

**RL Algorithms:**
- Stable Baselines 3 & SB3-Contrib - Provides PPO and Recurrent PPO implementations.

## Data Storage

**Databases:**
- None detected.

**File Storage:**
- Local Filesystem: Used for storing track boundaries (`.npz`), waypoints (`.npy`), and USD assets (`.usda`).
  - Cache: `arcproLab/mdp/track_boundaries_1x.npz`
  - Waypoints: `arcproLab/mdp/track_centerline_1x.npy`

**Caching:**
- `TrackManager` implements a local `.npz` cache to avoid slow USD marker collection on every startup.
- `RoadManager` discovers navigation gates from the USD stage and caches them during runtime.

## Authentication & Identity

**Auth Provider:**
- Custom - Local development environment; no external auth service detected.

## Monitoring & Observability

**Telemetry Logging:**
- 12-element Telemetry Vector: Implemented in `arcproLab/mdp/observations.py` via `get_telemetry_vector`.
- Telemetry Window: Real-time UI visualization in `arcproLab/mdp/visual_analytics.py`.

**Error Tracking:**
- None.

**Logs:**
- Tensorboard: Used for tracking training progress.
  - Config: `tensorboard_view_docker/`
- Local logs: Stored in `logs/` or `arcproLab/logs/`.
- Telemetry log: Dedicated logging via `relaunch_with_telemetry.sh`.

## CI/CD & Deployment

**Hosting:**
- Local workstations or GPU clusters.

**CI Pipeline:**
- GitHub Actions: `.github/workflows/ci.yml`.

## Environment Configuration

**Required env vars:**
- `ISAAC_PATH`: Path to Isaac Sim installation (standard requirement).

**Secrets location:**
- Not applicable.

## Webhooks & Callbacks

**Incoming:**
- None.

**Outgoing:**
- None.

---

*Integration audit: 2026-05-16*
