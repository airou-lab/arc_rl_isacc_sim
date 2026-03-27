# External Integrations

**Analysis Date:** 2024-05-23

## APIs & External Services

**Simulation Platform:**
- NVIDIA Isaac Sim / Omniverse USD - Provides the simulated environment for robot training and verification.
  - SDK/Client: Isaac Lab (inferred from codebase structure and project name)
  - Auth: Not explicitly detected, likely managed via Omniverse Launcher or local setup.

## Data Storage

**Databases:**
- None detected.

**File Storage:**
- Local filesystem for models (`arcproLab/models/road_following_model.pth`), track data (`arcproLab/mdp/track_centerline.npy`), and USD assets (`openStreetUSD/`).

**Caching:**
- None detected.

## Authentication & Identity

**Auth Provider:**
- None detected. Local development setup.

## Monitoring & Observability

**Error Tracking:**
- None detected.

**Logs:**
- Standard Python logging or print statements to console. No dedicated logging framework detected.

## CI/CD & Deployment

**Hosting:**
- Local development environment.
- CI/CD inferred from `.github/workflows/ci.yml` - suggests GitHub Actions for CI.

**CI Pipeline:**
- GitHub Actions (via `.github/workflows/ci.yml`) - for continuous integration (testing, linting).

## Environment Configuration

**Required env vars:**
- None explicitly detected in code (e.g., `os.environ`). Configuration likely handled through command-line arguments or internal module imports.

**Secrets location:**
- Not applicable for this project's current scope (no external services requiring secrets).

## Webhooks & Callbacks

**Incoming:**
- None detected.

**Outgoing:**
- None detected.

---

*Integration audit: 2024-05-23*
