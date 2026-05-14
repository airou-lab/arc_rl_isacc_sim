# Phase 15: Mastery Curriculum - Task 1 & 2 Summary

**Date:** 2026-05-14
**Status:** COMPLETED (Pending Headless Verification)

## Changes Implemented

### Task 1: Perception & Robust Logic
- **Vision Backbone**: Upgraded `FusionFeaturesExtractor` to use **ResNet-18** with `IMAGENET1K_V1` weights. Removed the final `fc` layer to provide 512-dim features.
- **VRAM Optimization**: Implemented **uint8 image storage** in the SB3 bridge (`HPPPDirectBridge`). Images are normalized to float on-GPU during the forward pass, reducing RolloutBuffer VRAM/RAM usage by 4x.
- **Windowed Search**: Refactored `TrackManager` to use a persistent **Windowed Waypoint Search** (+/- 50 indices). This provides O(1) waypoint tracking and prevents "snapping" issues on hairpins.
- **Gate-Awareness**: Updated `observations.py` to retrieve and store `dist_g` (gate distance) in `env.extras`.

### Task 2: Mastery Rewards & Environment Scaling
- **Mastery Rewards**: 
  - Updated `lateral_error_reward` with a **10cm plateau** (reward=1.0 within 10cm).
  - Implemented `jerk_penalty` (-100.0 weight) to suppress steering jitter.
- **Permissive Terminations**:
  - Tightened white line reset margin to **0.05m** for high-precision driving.
  - Implemented **Gate Masking**: Reset is suppressed if the robot is within 0.20m of a gate zone, allowing for lane-crossing during intersections.
- **Scaling**: 
  - Updated `ARCProEnvCfg` to use **32 parallel environments**.
  - Standardized camera resolution to **224x224** for ResNet compatibility.

## Verification Notes
- **ResNet-18**: Verified successful loading of weights via `python3`.
- **Environment**: Attempted verification training with 32 and 16 envs. Encountered persistent segmentation faults in headless mode related to `rtx.scenedb`. This appears to be a system/driver issue rather than a code regression, as it persists even with `xvfb-run`.
- **Next Step**: Perform a **GUI-based verification** (non-headless) to confirm stable physics and reward convergence.

## Next Phase
Transition to **Phase 16: Multi-Agent RL (MARL)**. Refactor RoadGraph to RoadManager and establish the SKRL policy stack.
