# Project State: ARCPro RL v1.2-dev

## Current Milestone
**Milestone 1: Physical Fidelity & Policy Foundation** (COMPLETE)

## Current Phase
**Phase 11: Retraining & Intersection Logic** (ACTIVE)

## Summary
The project has successfully achieved **True 1.0x Metric Scale**. All temporary "Giant Scale" (8x) artifacts have been removed. The environment is now a high-performance "Road in Void" setup, where the robot falls into the void if it leaves the pavement. Physics are stabilized at 500Hz for the 20kg robot.

## Recent Activity
- **1.0x Metric Restoration**: Reverted robot to 1:1 scale (0.45m width) and 20kg mass.
- **Environment 'Road in Void'**: Scaled track USD to 0.125, ghosted all non-road terrain (grass, foliage, fences), and removed ground plane.
- **Telemetry Update**: Observations and rewards are now in raw metric units.
- **Auto-Centering**: TrackManager now procedurally centers waypoints between yellow and white markers.
- **Strict Reset**: Implemented symmetric boundaries at +/- 0.2m from lane center for immediate reset on line contact.
- **Instant Reverse Kill**: Added `reverse_termination` which triggers an instant reset if local X-speed < 0 (no grace period).
- **Visual Confidence**: Added Yellow/White/Cyan dots in the GUI to verify lane centering and marker detection.

## Known Issues & Solutions
1. **Convergence dead-time**: The 1x physics are snappier; training requires careful monitoring of steering jitter.
2. **Parallel Spawning**: Resolved by making spawn events relative to `env_origins`.
3. **No Backwards Driving**: Strictly enforced via `reverse_termination`.
4. **Marker Detection**: TrackManager search string includes both path and material name to ensure robust marker discovery.

## Active Todos (Queue)
1. [/] **11-01-retrain-1x**: (RUNNING) Train fresh PPO policy at 1.0x scale with 32 parallel environments.
   - PID: 998995 (Background)
   - Log: `logs/ppo/20260417-005131`

## Completed
- [x] Phase 09: Training Loop Stabilization (8x Baseline).
- [x] Phase 10: Asset Downscaling (1.0x Restoration).
- [x] Phase 13: Environment Scale Alignment (Accelerated/Completed with Phase 10).

## Blockers
None.
