# Phase 2 Plan: Isaac Lab Migration (Parallel Training)

## Objective
Migrate the single-robot Direct API environment to Isaac Lab to enable multi-robot vectorized training, significantly increasing training throughput and resolving stability issues.

## Tasks
- [ ] Define Isaac Lab `Asset` schemas for the 34-joint ARCPro robot.
- [ ] Implement `ARCProTask` inheriting from `DirectRLEnv` or `ManagerBasedRLEnv` in Isaac Lab.
- Port vision-based lane reward logic to the Isaac Lab task.
- [ ] Configure `Hydra` for experiment management and scaling.
- [ ] Train a vectorized policy with 128+ parallel robots.

## Success Criteria
- Environment loads without Segmentation Faults.
- Multi-robot vectorization achieved (128+ robots).
- Training time for 1M steps reduced by >10x compared to Direct API.
