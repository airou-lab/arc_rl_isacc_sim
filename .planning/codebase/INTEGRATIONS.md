# External Integrations

**Analysis Date:** 2025-05-20

## APIs & External Services

**NVIDIA Isaac Sim / Kit:**
- Service: Simulation environment for physics and rendering.
- SDK: `omni.isaac.core`, `omni.kit.app`.

## Data Storage

**USD Framework:**
- Type: Universal Scene Description (USD) for environment and robot state.
- Connection: `omni.usd.get_context().get_stage()`.
- Primary file: `openStreetUSD/no_graph_sim_clean_1x.usda`.

**Waypoints:**
- Type: Pre-generated track geometry stored in `.npy` format.
- Client: `numpy`.
- Path: `arcproLab/mdp/track_centerline_1x.npy`.

## Authentication & Identity

**Auth Provider:**
- None (Local development environment only).

## Monitoring & Observability

**TensorBoard:**
- Approach: Integrated via SB3's `tensorboard_log` parameter.
- Usage: Training reward, loss, and episode length tracking.

**In-Sim Telemetry:**
- Approach: Custom `omni.ui` overlay for real-time verification.
- Location: `arcproLab/mdp/visual_analytics.py`.

## CI/CD & Deployment

**CI Pipeline:**
- Service: GitHub Actions (via `.github/workflows/ci.yml`).
- Purpose: Basic syntax and configuration checks.

**Hosting:**
- Platform: Local GPU workstation or headless server.

## Environment Configuration

**Required env vars:**
- `ISAACSIM_PATH`: Path to the Isaac Sim installation.
- `LD_LIBRARY_PATH`: Standard CUDA and NVIDIA library paths.

## Webhooks & Callbacks

**Incoming:**
- None.

**Outgoing:**
- None.

---

*Integration audit: 2025-05-20*
