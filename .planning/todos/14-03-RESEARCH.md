# Research: Telemetry Curvature (14-03)

## Goal
Calculate and expose upcoming path curvature (`obs[:, 10]`) so the RL policy can proactively steer into upcoming turns instead of purely reacting to immediate lateral errors.

## Codebase Analysis
- **`arcproLab/mdp/track_manager.py`**: Loads waypoints from `track_centerline_1x.npy`. The waypoints have a shape of `(N, 3)` representing `[X, Y, Yaw]`. The spacing between consecutive waypoints is ~0.052 meters.
- **`arcproLab/mdp/observations.py`**: Telemetry vector is defined in `get_telemetry_vector`. Currently, `obs[:, 10] = 0.0` is a hardcoded placeholder.

## Technical Steps
1. **Implement `compute_curvature` in `TrackManager`**:
   Add a new method `def compute_curvature(self, pos: torch.Tensor, lookahead_dist: float = 2.0) -> torch.Tensor:`
   - Find the closest waypoint index for each robot position (similar to `compute_errors`).
   - Calculate the future index by advancing a fixed number of steps. Since waypoint spacing is ~0.052m, a 2.0m lookahead is roughly `lookahead_steps = int(2.0 / 0.052) \approx 38` indices.
   - Use modulo arithmetic to wrap around the track: `future_idx = (closest_idx + lookahead_steps) % len(self.waypoints)`.
   - Calculate the change in heading: `d_theta = self.waypoints[future_idx, 2] - self.waypoints[closest_idx, 2]`.
   - Normalize `d_theta` to `[-pi, pi]`.
   - Calculate exact distance `d_s = torch.norm(self.waypoints[future_idx, :2] - self.waypoints[closest_idx, :2], dim=-1)`.
   - Compute curvature: `kappa = d_theta / d_s`. (Handle edge cases where `d_s` is zero by clamping or adding epsilon).

2. **Integrate into `observations.py`**:
   - Inside `get_telemetry_vector`, call `kappa = tm.compute_curvature(local_pos)`.
   - Assign the result to the observation tensor: `obs[:, 10] = kappa`.

## Edge Cases and Risks
- **Lookahead Resolution**: The fixed index advance assumes uniform waypoint spacing. If waypoints have drastically varying distances, it's safer to iteratively search for the first waypoint `> 2.0m` away, though that is slower in PyTorch. Fixed index is highly efficient and acceptable for ~0.05m uniform tracks.
- **Directionality Convention**: Depending on whether left turns are positive or negative yaw, the sign of `kappa` must consistently match the steering action space.
- **Noise / Discontinuity**: At the start/end seam of the closed-loop track, modulo arithmetic correctly wraps the index, but normalizing `d_theta` is crucial so a jump from `-pi` to `+pi` doesn't result in an extreme curvature spike.
