# Research: TrackManager Caching (14-02)

## Goal
Eliminate the 5-10 second blocking delay at environment startup by caching the extracted USD geometry points for track boundaries and gates.

## Codebase Analysis
- **Component**: `arcproLab/mdp/track_manager.py`
- **Current Flow**: 
  In `TrackManager.ensure_synced()`, it currently always calls `self.collect_raw_marker_points()` on the first sync (`self.sync_attempts == 0`).
  This function traverses the USD stage and parses meshes into numpy arrays (`self.raw_yellow_pts`, `self.raw_white_pts`, `self.raw_gate_pts`).

## Technical Steps
1. **Define Cache Paths**: In `TrackManager.__init__`, define cache file paths (e.g., `self.yellow_cache = os.path.join(os.path.dirname(__file__), "track_yellow_1x.npy")`).
2. **Check for Rebuild Flag**: Check `sys.argv` for a `--rebuild_track_cache` flag.
3. **Load from Cache**: In `ensure_synced()`, before running USD collection, check if the `.npy` files exist.
   - If they exist (and rebuild is false): Use `np.load()` to populate `self.raw_yellow_pts`, `self.raw_white_pts`, and `self.raw_gate_pts`.
   - If they do not exist: Proceed with `self.collect_raw_marker_points()`.
4. **Save to Cache**: At the end of `collect_raw_marker_points()`, add logic to save the computed `np.ndarray` results to disk using `np.save(path, array)`.

## Edge Cases and Risks
- **None Arrays**: If `finalize_group` returns `None` (e.g., no gates found), the script should handle that gracefully (either save an empty array or skip saving, and handle it on load).
- **Environment Origin Context**: The points in `collect_raw_marker_points` are transformed relative to `env0_origin`. As long as the environment origin (env_0 position) remains deterministic, this relative cache is valid. 
- **Scale Changes**: If Todo 14-01 changes the USD scaling, the cache will be invalidated. `--rebuild_track_cache` MUST be run after 14-01 is complete to ensure points match the flattened map.
