# Todo: Telemetry Curvature (Kappa) Placeholder

**Priority**: High
**Status**: Queue
**Component**: `arcproLab/mdp/observations.py` and `arcproLab/mdp/track_manager.py`

## Problem
Index 10 of the telemetry vector (`obs[:, 10]`) represents the "Path Curvature" (Kappa) ahead of the robot. Currently, this is hardcoded to `0.0` in `observations.py`. Because of this placeholder, the policy cannot "see" upcoming turns and can only react once a lateral error has already occurred.

## Tasks
- [x] In `TrackManager`, implement a method to compute the curvature of the path a certain distance (lookahead horizon) ahead of the robot.
- [x] The curvature $\kappa$ can be approximated using the change in heading over distance ($d\theta / ds$) between the closest waypoint and a future waypoint (e.g., 2 meters ahead).
- [x] In `observations.py`, call this new `TrackManager` method and assign the calculated value to `obs[:, 10]`.


## Verification
- Print the value of `obs[0, 10]` during GUI verification.
- The value should be near `0.0` on straightaways.
- The value should become positive (or negative, depending on convention) as the robot approaches a curve.