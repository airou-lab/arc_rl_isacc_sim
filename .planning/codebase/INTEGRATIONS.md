# External Integrations

**Analysis Date:** 2025-03-21

## APIs & External Services

**Simulation:**
- NVIDIA Omniverse Isaac Sim - Simulation engine providing physics (PhysX) and rendering (RTX).
- Isaac Lab v1.0.1 - Integration layer for RL tasks, environments, and managers.

**RL Algorithms:**
- Stable Baselines3 (SB3) - Third-party RL library integrated via `Sb3VecEnvWrapper` for training PPO policies (`arcproLab/scripts/train_policy.py`).

## Data Storage

**Databases:**
- Not detected.

**File Storage:**
- Local filesystem for checkpointing models (`logs/ppo/`) and loading pre-trained weights (`arcproLab/models/road_following_model.pth`).
- USD files for static assets and environment scenes (`openStreetUSD/`, `arcproLab/assets/`).

**Caching:**
- None detected.

## Authentication & Identity

**Auth Provider:**
- None required for current local development.

## Monitoring & Observability

**Error Tracking:**
- Custom `TrackManager` for computing lateral and heading errors relative to a track centerline (`arcproLab/mdp/track_manager.py`).

**Logs:**
- TensorBoard integration via Stable Baselines3 for training progress monitoring (`train_policy.py`).
- Console output and custom UI windows for real-time telemetry audit (`arcproLab/mdp/visual_analytics.py`).

## CI/CD & Deployment

**Hosting:**
- Local NVIDIA RTX workstations.

**CI Pipeline:**
- GitHub Actions for linting and unit testing (`.github/workflows/ci.yml`).

## Environment Configuration

**Required env vars:**
- None detected. Configuration is primary class-based within the codebase (`arcproLab/arcpro_env_cfg.py`).

**Secrets location:**
- Not applicable. No external API keys or secrets used in the current version.

## Webhooks & Callbacks

**Incoming:**
- None.

**Outgoing:**
- None.

---

*Integration audit: 2025-03-21*
