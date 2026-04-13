# Project State: ARCPro RL v1.2-dev

## Current Phase
**Phase 10: 1.0x Metric Restoration (COMPLETE)**

## Summary
The project has successfully reverted from the temporary 8x "Giant Scale" back to the standard **1.0x Metric Scale** (F1Tenth). This transition eliminates complex normalization math and ensures the simulation matches physical reality. Grass, foliage, and other visual clutter have been removed from the environment to focus on road navigation.

## Recent Activity
- **1.0x Metric Scale Restoration**: Reverted simulation to original F1Tenth scale (0.45m robot). Removed observation normalization (0.125) and giant mass (1500kg -> 20kg).
- **Environment Cleanup**: Removed grass, foliage, and fences from the track USD to simplify visuals and focus on road features.
- **Physics Stabilization**: Re-calibrated actuator gains and termination thresholds for stable 1x physics.
- **Waypoint Alignment**: Scaled all centerline waypoints to 1x to match the new world dimensions.

## Key Achievements
- **Stable 1x Simulation**: Confirmed physics stability at small scale through dimension verification.
- **Clean Visuals**: Environment is now strictly road and markers, improving visual clarity for the agent.
- **Normalized Observations**: Speed and lateral error are now expressed in raw meters and meters/sec.

## Completed (Phase 10 - Asset Downscaling)
- [x] Global Track Downscale (0.125 factor).
- [x] Robot Scale Revert (1.0x).
- [x] Physics Calibration (20kg mass, realistic gains).
- [x] Waypoint Scaling (1x).
- [x] Termination & Reward Re-calibration.
- [x] Grass & Foliage Removal.

## Next Step (Phase 11)
- [ ] **Retraining at 1x**: Initiate fresh PPO training run to adapt to 1x physics.
- [ ] **Intersection Planning**: Implement logic for handling the intersection waypoints.
