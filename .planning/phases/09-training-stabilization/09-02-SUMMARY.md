# Phase 09-02 Summary: Physical Calibration & AWD Fix

## Goal
Stabilize the 8.0x scale robot physics and correct AWD joint mapping to ensure a reliable training baseline.

## Status
**COMPLETE**

## Key Achievements
- **1500kg Physics**: Successfully transitioned from 20kg "balloon" physics to a 1500kg heavy chassis model.
- **AWD Alignment**: Corrected joint mapping (Index 2-5) and normalized wheel direction signs for forward motion.
- **Mathematical Reset**: Implemented a 0.6m lateral drift limit using `TrackManager` math, which proved more robust than contact-based sensing for this specific map scale.
- **Telemetry Sync**: Aligned speed and lateral error observations with a 0.125 normalization factor.

## Deviations from Plan
- **Contact Sensing**: Pivoted from `ContactSensorCfg` to direct mathematical drift limits. Contact sensors on large meshes in Isaac Sim can be jittery; the `TrackManager` waypoints provided a cleaner, more consistent signal for the 3.6m wide robot.
- **Environment Count**: Reduced from 2 environments to 1 to bypass coordinate drift issues in vectorized Isaac Lab setups.

## Verification
- Verified 1000 steps of stable forward driving in GUI.
- Math centerline (X=-130.03) confirmed aligned with visual yellow line.
