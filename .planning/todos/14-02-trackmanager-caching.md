# Todo: TrackManager Performance Bottleneck

**Priority**: Medium
**Status**: Queue
**Component**: `arcproLab/mdp/track_manager.py`

## Problem
Every time the environment is initialized (or `TrackManager` is instantiated without a cache), it scans the entire USD stage, extracting thousands of vertices for yellow lines, white lines, and gates. This introduces a 5-10 second blocking delay at startup.

## Tasks
- [x] Modify `TrackManager.collect_raw_marker_points()` to export the extracted `raw_yellow_pts`, `raw_white_pts`, and `raw_gate_pts` as numpy arrays (`.npy`) or PyTorch tensors (`.pt`) to disk.
- [x] In `TrackManager.ensure_synced()`, add logic to check if these cache files exist.
- [x] If the cache exists, load the points directly from disk, bypassing the expensive USD scene traversal.
- [x] Add a `--rebuild_track_cache` CLI flag (or similar mechanism) to force a re-calculation if the USD map ever changes.


## Verification
- Environment startup time significantly decreases (by several seconds).
- Visual debug mode (`--debug`) still correctly draws all spheres.
- Boundary terminations continue to function properly during training.