# Project State: ARCPro RL v1.1-dev

## Current Phase
**Phase 5: Policy Integration (Hierarchical Architecture)**

## Summary
The simulation environment is stable and localized (v1.1-dev). We have completed a comprehensive repository cleanup, moving utility scripts to `tools/` and localizing essential assets from the removed `f1tenth_trainer` submodule. The environment is now ready for the integration of the **Hierarchical Path Planning Policy**.

## Recent Activity
- **Repository Reorganization**: Moved ~80 utility scripts to `arcproLab/tools/` to clean up the workspace.
- **Asset Localization**: Removed `f1tenth_trainer` submodule and moved `F1Tenth_Generated.usd` and `road_following_model.pth` into the `arcproLab/` internal structure.
- **Stable Verification**: Confirmed that the "v1.0.1 baseline" (SB3 Lane Following) still works perfectly after the reorganization.
- **Config Hardening**: Updated `arcpro_robot_cfg.py` and `arcpro_env_cfg.py` to point to local assets and use verified 20x scaling.

## Key Achievements
- **Clean Workspace**: Removed redundant external dependencies and script clutter.
- **Stability Maintained**: Verified the robot spawns, falls, and drives without regressions.

## Next Actions
- [ ] Implement 'lane-aligned' spawning logic in `events.py`.
- [ ] Implement 12-float telemetry protocol in `observations.py`.
- [ ] Implement 'Hybrid Racer' Gaussian rewards in `rewards.py`.
- [ ] Integrate Hierarchical Path Planning Policy components from `arc_rl_isacc_policy`.
