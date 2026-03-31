# External Integrations

**Analysis Date:** 2024-10-24

## APIs & External Services

**Simulation:**
- NVIDIA Isaac Sim - Primary simulation platform for robotics.
  - SDK/Client: `isaaclab` and `omni` Python packages.
  - Auth: Local license / Omniverse Nucleus (if used).

**Visualization:**
- Tiled Camera Sensor - In-sim camera used for visual observations in `arcproLab/arcpro_env_cfg.py`.
  - Data types: RGB, 160x90 resolution.

## Data Storage

**Databases:**
- None detected.

**File Storage:**
- USD (Universal Scene Description) - Used for all 3D assets (track and robot).
  - Track: `openStreetUSD/no_graph_sim_final.usd`.
  - Robot: `arcproLab/assets/robot/F1Tenth_Metric.usd`.
- NumPy (`.npy`) - Used for waypoints in `arcproLab/mdp/track_centerline.npy`.

**Caching:**
- Track Manager Cache - `TrackManager` samples and saves waypoints to `.npy` to avoid re-sampling the USD stage.

## Authentication & Identity

**Auth Provider:**
- NVIDIA Omniverse - Handles license authentication for the simulator.

## Monitoring & Observability

**Error Tracking:**
- None detected.

**Logs:**
- Console - Extensive use of `print()` for status and warning messages in `arcproLab/mdp/track_manager.py`.

## CI/CD & Deployment

**Hosting:**
- Local workstation or GPU cluster (Slurm/Docker).

**CI Pipeline:**
- GitHub Actions - `.github/workflows/ci.yml` (present in repository root).

## Environment Configuration

**Required env vars:**
- `CUDA_VISIBLE_DEVICES` - Specifies GPU to use.
- `ISAAC_SIM_PATH` - (Implicit) Path to Isaac Sim installation.

**Secrets location:**
- Not applicable.

## 1.0x Metric PhysX Configuration

The codebase is explicitly configured for 1.0x metric scale physics to ensure high-fidelity simulation of the F1Tenth vehicle.

**Global PhysX (arcproLab/arcpro_env_cfg.py):**
- **Solver:** TGS (Temporal Gauss-Seidel) - `solver_type=1`.
- **Iteration Count:** 8 position, 4 velocity.
- **Time Step (`dt`):** 0.005s (200Hz).
- **Collision:** CCD (Continuous Collision Detection) enabled.
- **GPU Buffers:** Explicitly configured high-capacity buffers (e.g., `gpu_max_rigid_contact_count=2**21`).

**Robot Articulation (arcproLab/arcpro_robot_cfg.py):**
- **Precision Solver:** 32 position iterations, 16 velocity iterations.
- **Scale:** `(1.0, 1.0, 1.0)` (True metric scale).

## Webhooks & Callbacks

**Incoming:**
- None detected.

**Outgoing:**
- None detected.

---

*Integration audit: 2024-10-24*
