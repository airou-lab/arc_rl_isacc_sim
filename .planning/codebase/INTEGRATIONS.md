# External Integrations

**Analysis Date:** 2025-05-21

## APIs & External Services

**NVIDIA Isaac Sim / Kit:**
- Service: Simulation engine for physics, rendering, and USD management.
- SDK: `omni.isaac.core`, `omni.kit.app`.

## Internal Submodules

**ARCPro RL Policy Stack (`arcproLab/policy_stack`):**
- Purpose: Shared repository for advanced policy logic, hierarchical architectures, and ROS2 deployment wrappers.
- Syncing: Managed as a git submodule to ensure parity between simulation and physical hardware deployment.

## Data Storage

**USD Framework (Primary Source of Truth):**
- Type: Universal Scene Description (USD) for environment, markers, and robot state.
- Connection: `omni.usd.get_context().get_stage()`.
- Primary file: `openStreetUSD/no_graph_sim_clean_1x.usda`.

**Marker Points:**
- Type: Runtime collection of mesh vertices from the USD stage (filtered by "yellow" or "white" materials/paths).
- Handling: Vectorized in `TrackManager` for real-time proximity math using `torch.cdist`.

**Legacy Waypoints:**
- Type: Static `.npy` files previously used for centerline generation (Now deprecated).
- Path: `arcproLab/mdp/track_centerline_1x.npy`.

## Authentication & Identity

**Auth Provider:**
- None (Local development environment only).

## Monitoring & Observability

**TensorBoard:**
- Approach: Integrated via SB3's `tensorboard_log` parameter.
- Usage: Training reward, loss, and episode length tracking.

**In-Sim Telemetry:**
- Approach: Custom `omni.ui` overlay for real-time verification and boundary visualization.
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

*Integration audit: 2025-05-21*
