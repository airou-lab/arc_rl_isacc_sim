# Project State: ARCPro RL v1.1-dev

## Current Phase
**Phase 7: Revert to True Physics Mode**

## Summary
The project has shifted focus to reverting simulation workarounds. We are transitioning from "Giant Scale" (20x) back to "True Physics" (1.0x metric) using a corrected robot asset (`F1Tenth_Metric.usd`). Phase 5 (Policy Integration) is currently paused.

## Recent Activity
- **Repository Reorganization**: Moved ~80 utility scripts to `arcproLab/tools/` to clean up the workspace.
- **Asset Localization**: Removed `f1tenth_trainer` submodule and moved `F1Tenth_Generated.usd` and `road_following_model.pth` into the `arcproLab/` internal structure.
- **Stable Verification**: Confirmed that the "v1.0.1 baseline" (SB3 Lane Following) still works perfectly after the reorganization.
- **Config Hardening**: Updated `arcpro_robot_cfg.py` and `arcpro_env_cfg.py` to point to local assets and use verified 20x scaling.

## Key Achievements
- **Clean Workspace**: Removed redundant external dependencies and script clutter.
- **Stability Maintained**: Verified the robot spawns, falls, and drives without regressions.

## Active Tasks (Phase 7)
- [x] Configure Environment Physics, Scaling, and Camera Offset in `arcpro_env_cfg.py`.
- [x] Configure Robot Asset in `arcpro_robot_cfg.py`.
- [x] Remove Robot Stability Event in `mdp/events.py`.
- [ ] Verify True Physics Environment Setup (Human Verification).

## Backlog (Phase 5)
- [ ] Implement 'lane-aligned' spawning logic in `events.py`.
- [ ] Implement 12-float telemetry protocol in `observations.py`.
- [ ] Implement 'Hybrid Racer' Gaussian rewards in `rewards.py`.
- [ ] Integrate Hierarchical Path Planning Policy components from `arc_rl_isacc_policy`.
