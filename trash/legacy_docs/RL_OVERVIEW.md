# ARCPro Reinforcement Learning - Legacy Overview

> **STATUS:** DEPRECATED in favor of Phase 3 (Isaac Lab). This document describes the original Direct API architecture.

## Project Architecture (Legacy)

- **`arc_rl_isacc_sim/`**: Original Isaac Sim environment logic and digital twin assets.
- **`arc_rl_isacc_policy/`**: Policy definitions and training protocols used during initial development.

## Migration to Isaac Lab
Due to stability issues with high-fidelity articulations in the Direct API (Segmentation Faults), the system is being migrated to Isaac Lab.

- **Primary Logic:** Porting from `isaac_direct_env.py` to `ManagerBasedRLEnv`.
- **Target Throughput:** 2,000+ FPS (Massive Vectorization).
- **Verified Asset:** `F1Tenth_Metric_Baked.usd` (Repaired joint local positions).
