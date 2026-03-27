# Technology Stack

**Analysis Date:** 2024-05-23

## Languages

**Primary:**
- Python 3.x - Core logic, RL implementations, scripting

**Secondary:**
- Not detected

## Runtime

**Environment:**
- Python 3.x (specific version depends on environment setup, implied by requirements.txt)

**Package Manager:**
- pip
- Lockfile: requirements.txt is present

## Frameworks

**Core:**
- PyTorch (torch) - Deep learning framework for RL policy training and execution
- NumPy (numpy) - Numerical operations, array manipulation

**Testing:**
- Pytest (pytest) - Unit and integration testing

**Build/Dev:**
- Not explicitly managed by a dedicated build tool, standard Python development practices.

## Key Dependencies

**Critical:**
- torch - Fundamental for defining, training, and running neural network policies.
- numpy - Essential for numerical computations within the RL environment and agent.

**Infrastructure:**
- Isaac Sim / Omniverse USD - Inferred from the project name (arc_rl_isacc_sim) and the presence of `.usd` files (e.g., `openStreetUSD/arcpro_RL_open_street_sim.usd`). This is the simulation platform where the RL agent operates. (Note: Isaac Sim itself is not listed in `requirements.txt` as it's an external platform/SDK rather than a pip package in this context).

## Configuration

**Environment:**
- Python environment managed by `venv` (indicated by `.venv/` and `venv/` directories).
- Dependencies are listed in `requirements.txt`.
- No explicit application-specific configuration files (e.g., `.ini`, `.yaml`) detected beyond Python modules themselves.

**Build:**
- Standard Python packaging practices. No custom build config files detected.

## Platform Requirements

**Development:**
- Python 3.x compatible environment.
- NVIDIA GPU with CUDA for PyTorch acceleration (implied by RL/deep learning context).
- Isaac Sim installation for full simulation capabilities.

**Production:**
- Deployment target likely an environment with Python 3.x and NVIDIA hardware capable of running Isaac Sim or a deployed policy.

---

*Stack analysis: 2024-05-23*
