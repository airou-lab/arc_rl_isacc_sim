# CTA RESUME HERE

## Active Execution State
- **MILESTONE**: M004: RealSense Realistic Kinematics & Clean Step 0 Bootstrap
- **Phase**: Phase 03: Step 0 Scratch Training & Corner 1 Transit
- **Status**: High episode length (~1.0k–1.5k steps) baseline snapshot preserved; marked as NOT WORKING for Corner 1 transit (vehicle fails to negotiate Corner 1 and drives off/crashes in GUI).
- **Next Todo**: Formulate and test alternative cornering/control solutions from this stable high-episode-length baseline.
- **Checkpoint Timestamp**: 2026-09-21T21:30:00Z

## Persistent State Data Sources
- **Turn Actions Database**: `.cta/cta_turns.db`
- **Codebase RAG Database**: `.cta/cta_codebase.db`
- **Codebase Index Guide**: `cta_codebase_index.yml`
- **Directory Structure**: `cta_directory_structure.yml`

## Recent Completed Actions
- **ROLLBACK**: Restored working tree to morning baseline snapshot (reverted afternoon experimental deviations in skrl_models, observations, track_manager, and terminations).
- **VERIFICATION**: Headless execution confirmed clean initialization, straight tracking, and 333+ waypoints traversed.
- **DIAGNOSIS**: In GUI evaluation, policy achieves high episode lengths on straights but fails to execute Corner 1 turning envelope.

## Open Issues, Concerns & Learnings
- **DECISION** (Issue 110 Resolution: Lookahead Curvature Window & Track Direction De-Latching):
  1) Swarm Audit on Checkpoint Rollback: Certified NO ROLLBACK. `agent_3550000.pt` (71.9M transitions) reached all-time peak episode lengths of 1,888 steps, rewards of +11,683.1, and 773 cumulative waypoints. The failure in `debug.sh` where the car crossed the yellow line into the void was caused by feedforward telemetry math outside the neural weights.
  2) Root Cause A (Single-Point Lookahead Blindspot): `get_lookahead_curvature` sampled only a single point at +1.6m (160 WPs). Because Corner 1 is only 1.95m long (150 WPs), when the vehicle reached the turn entry/apex (WP 880-940), the +1.6m point had already reached the straightaway of Leg 2 (WP 1040-1100), dropping `lookahead_kappa` to 0.08 rad/m and collapsing feedforward steer bias from +0.40 to +0.04 and brake bias to zero.
  3) Root Cause B (`track_dir` Latching Corruption): In `observations.py` and `terminations.py`, post-crash vehicle poses with yaw deflection > 90 deg triggered `raw_head_err > 1.5708`, latching `track_dir = -1.0`. This caused `get_lookahead_curvature` to sample 1.6m BEHIND the vehicle on the start straight (kappa=0) and invert steering sign, commanding full straight throttle into the intersection.
  4) Fix Implemented:
     - Implemented strided forward window search (`stride=4`, offsets 0 to 160) in `TrackManager.get_lookahead_curvature`, returning peak signed curvature across the lookahead horizon. Guarantees continuous +0.825 rad/m curvature, +0.40 left turn bias, and 0.289 braking bias throughout the approach and entire corner arc.
     - Sanitized `track_dir` to constant +1.0 in `observations.py` and `terminations.py` across all environments and resets.
     - Preserved `track_centerline.npy` and `+ 3.14159265` yaw offset, maintaining 100% mathematical integrity of `lat_err`, `head_err`, rewards, terminations, and Critic state value functions.
- **LEARNING** (Kinematic Throttle Pre-Braking Prior for Corner 1 Breakthrough): On sharp corners (R=0.464m, kappa=2.155 m^-1), unconstrained PPO policies exploit straightaway sprint velocity (0.80-0.95 m/s) because waypoint progress reward dominates over curvature speed reward. Adding an inductive deceleration prior on the drive channel (brake_bias = clamp(abs(kappa)*0.35, 0.0, 0.70)) smoothly scales drive commands down to ~11 rad/s (0.35 m/s) ahead of the apex, allowing the vehicle to negotiate the 90-degree turn within linear tire friction limits. Coupling this with 120.0 speed tracking reward and 50% Sector 2 curriculum spawns provides continuous gradient for post-apex Leg 2 exploration.
- **DECISION** (Sentinel 2 100k Evaluation Window & 800 Step Penetration Gate): Expanded Corner 1 Apex Cliff Sentinel from 50k to 100k steps and set progress threshold to 800 steps. In the surging run at step 795k+, agents achieved peak episode lengths of 818 steps with rewards exceeding 4,500. The previous 50k window with <= 860 threshold was prematurely resetting this high-performing run. Now, reaching >= 800 steps safely resets the evaluation window, giving the policy continuous runway to master Corner 1 exit.
- **DECISION** (Maintain agent_900000.pt Starting Point): Evaluated rollback options: rolling back to Step 0 would discard 900k steps of visual lane centering and forward dynamics, while rolling back to 1.15M would reintroduce pre-feedforward understeer bias. agent_900000.pt with kinematic feedforward prior (0.485*kappa) and differential LR (3e-4 head / 3e-5 backbone) is actively achieving max episode lengths >630, rew >3600, and wps >220. Sentinel 3 false-positive restart loop was identified as the root cause of apparent stagnation and has been fixed.
- **LEARNING** (Sentinel Safeguards Audit for Early Learning): 1. Sentinel 3 ('WALL_HUGGING') at line 426 triggered repeatedly (9 times between 22:04 and 00:07) because it lacks an activation step gate. A newborn policy at step 50k naturally exhibits avg_ep_len ~135-145 steps while learning to navigate curves from the 5-sector spawner (especially Sector 2 with 2.5m runway to C1). With avg_speed ~0.62 m/s and avg_rew ~540, it satisfied all 3 conditions: ep_len < 150, speed >= 0.50, rew < 600. However, telemetry confirms the policy was healthy and actively advancing (wps_cum=50-78, max_len=370, reaching 597 steps and 1010 rew by step 58k). Sentinel 3 MUST be gated with latest_batch.step >= 100_000 or 150_000 and should also check wps_cum < 10 to prevent false-positive restarts of newborn exploration. 2. Sentinel 1 (Crawling) is safe due to ep_len >= 150 gate. Sentinel 2 (Apex Cliff) is well-gated with step >= 300,000. Sentinel 4 (Collapse) and Sentinel 5 (Hang) are properly guarded and functioning as intended. 3. Training survived Sentinel 3 interventions due to tmux session self-guarding in start_tmux_training.sh preventing accidental session termination. 4. arcproLab/__init__.py top-level imports prevent importing AutonomousSentinel outside the Omniverse environment unless mocked.

## Resume Instructions
1. Reset or clear the screen safely using `/clear` or `/cta-clear`.
2. On fresh context, invoke `/cta-resume` to restore project continuity directly from this file and SQLite databases.
