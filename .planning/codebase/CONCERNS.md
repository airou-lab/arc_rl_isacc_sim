# Codebase Concerns

**Analysis Date:** 2024-05-13

## Tech Debt

**VRAM Efficiency (Explicit Disables):**
- Issue: `god_view` monitor and per-agent `tiled_camera` are disabled by default to save VRAM. This limits debugging visibility and potential visual-based learning features unless manually enabled with high-spec hardware.
- Files: `arcproLab/arcpro_env_cfg.py`
- Impact: Difficulty in visual inspection of agent behavior across multiple environments. High environment counts (128+) with cameras enabled may lead to OOM on GPUs with <12GB VRAM.
- Fix approach: Implement a "sparse camera" mode where only one or a few environments have cameras enabled for monitoring, rather than all-or-nothing.

**NaN Patching:**
- Issue: Reward functions and observation vectors use `torch.isnan` checks to zero out NaNs. This "patches" math instability (e.g., from physics or geometric calculations) rather than addressing the root cause.
- Files: `arcproLab/mdp/rewards.py`, `arcproLab/mdp/observations.py`
- Impact: Silent failure of signals. If a robot flies off-track and produces NaNs, the agent might receive a "0.0" reward instead of a clear penalty, leading to poor learning convergence.
- Fix approach: Identify the source of NaNs (likely `TrackManager.compute_errors` or physics-driven velocity spikes) and use `torch.clamp` or robust math (e.g., `atan2` safely) to prevent them.

## Physical Stability

**Low Simulation Frequency:**
- Issue: `SimulationCfg` uses `dt=1.0/60.0` (60Hz).
- Files: `arcproLab/arcpro_env_cfg.py`
- Impact: For high-speed vehicles, 60Hz might be insufficient for stable tire-track contact and precise steering joint control. This often results in "jittery" movement or robots clipping through the track.
- Fix approach: Increase simulation frequency to 120Hz or 240Hz, or use sub-stepping in PhysX.

**Hardcoded Stability Overrides:**
- Issue: `setup_robot_stability` manually overrides joint stiffness (1000.0) and damping (50.0/100.0).
- Files: `arcproLab/mdp/events.py`
- Impact: These "magic numbers" might hide underlying issues with the robot's USD mass properties or actuator definitions. They may not scale if the robot's mass or scale is changed.
- Fix approach: Properly tune the `ARCPRO_ROBOT_CFG` in the robot's asset definition rather than using a startup event for hard overrides.

**Environment Spacing:**
- Issue: `env_spacing=5.0` is relatively tight for a track-based environment.
- Files: `arcproLab/arcpro_env_cfg.py`
- Impact: If a robot deviates from the track, it may easily collide with the "track" asset of an adjacent environment clone, causing physics chaos.
- Fix approach: Increase spacing to 50.0m+ or use a single global track asset and only clone the robots.

## Metric Scaling

**Initial Reward Delay:**
- Issue: All reward terms are zeroed if `env.episode_length_buf < 20`.
- Files: `arcproLab/mdp/rewards.py`
- Impact: The agent receives no feedback for the first ~0.66s of an episode. This is likely intended to let the robot "settle" after spawning, but it wastes simulation time and complicates temporal credit assignment.
- Fix approach: Ensure robust spawning (e.g., slightly above ground with damping) so the robot is stable from step 0.

**Lateral Error Discontinuity:**
- Issue: Lateral error reward jumps from `+1.0` (if error < 0.5m) to `-abs(lat_err) * 2.0` (if error > 0.5m).
- Files: `arcproLab/mdp/rewards.py`
- Impact: A robot at 0.49m error gets +1.0, while at 0.51m it gets -1.02. This sharp penalty cliff can cause unstable gradients during training.
- Fix approach: Use a continuous function (e.g., Gaussian reward or a smooth hinge loss) for lateral error.

**Missing Accumulated Distance:**
- Issue: `get_telemetry_vector` attempts to read `env.extras["distance"]`, but this extra is not updated anywhere in the `mdp/` logic.
- Files: `arcproLab/mdp/observations.py`
- Impact: The observation index 11 remains 0.0, depriving the agent of a critical progress metric.
- Fix approach: Implement a `distance_travelled` term in the observation manager or an event that updates `env.extras["distance"]`.

## Fragile Areas

**TrackManager Waypoint Sampling:**
- Files: `arcproLab/mdp/track_manager.py`
- Why fragile: `sample_waypoints_from_usd` uses a heuristic search for "road" meshes and a Nearest Neighbor ordering that can break if the USD track has gaps or complex branching.
- Safe modification: Manually verify the generated `track_centerline.npy` whenever the USD track is updated.
- Test coverage: Gaps in verifying waypoint continuity.

## Scaling Limits

**GPU Memory (VRAM):**
- Current capacity: Tested on 128 envs (likely 16GB VRAM limit).
- Limit: Isaac Sim memory usage scales linearly with `num_envs` when using `TiledCamera`.
- Scaling path: For large-scale training (1024+ envs), cameras must remain disabled, or "flat" sensor alternatives must be used.

---

*Concerns audit: 2024-05-13*
