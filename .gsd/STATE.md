# GSD State

**Active Milestone:** M001: Milestone 3: HD Perception and Production Hardening
**Active Slice:** S01: S01
**Phase:** evaluating-gates
**Requirements Status:** 5 active · 0 validated · 0 deferred · 0 out of scope

## Milestone Registry
- 🔄 **M001:** Milestone 3: HD Perception and Production Hardening

## Recent Decisions
- **D008:** Resolved environment physics/camera orientation bugs (camera native backward offset vs drive direction) and pushed cleanup to `dev` branch.
- **D009:** Fixed 5 critical RL bugs (Suicide Bias, 3D Distance Z-Axis Boundary Bug, Distance Inflation, Double Stepping FSM, 1-Step Reward Delay).
- **D010:** (REVERTED) Injected 5 forward-looking waypoints into the telemetry vector. This was reverted because providing numerical anticipation completely bypassed the requirement (R002) for the agent to learn HD Vision-based navigation.
- **D011:** (SUPERSEDED) Implemented Balanced Transfer Learning in `fusion_policy.py`. We unfroze `layer4` because the agent seemed blind when the network was fully frozen. However, this still caused late-stage KL divergence spikes.
- **D012:** Discovered the TRUE root cause of the vision failure: the CNN normalization block in `fusion_policy.py` was bypassing ImageNet `mean`/`std` normalization for `[0, 1]` images. This fed unnormalized tensors into frozen ImageNet layers, producing garbage feature maps (rendering the agent blind). Fixed the normalization math and completely FROZE the entire ResNet-18 backbone again. The agent can now "see" the track using stable, frozen edge-detectors, and PPO is 100% stable.
- **D013:** Fixed the "Zoolander Bug". `ActionCfg` fields were sorted alphabetically by IsaacLab (`drive`, `steering`), meaning the `train_policy.py` bounds `[Steering, Throttle, Brake]` applied the brake's `[0.0, 1.0]` bound to the steering action, physically preventing the robot from turning right. Renamed fields to `a_steering` and `b_drive` to restore the correct order. Also reduced the stationary penalty from 1.0 to 0.05 to prevent the agent from intentionally crashing to avoid massive stagnation bleed.
- **D014:** Fixed "Reward Farming Exploit" & "Stagnation Bias". The agent refused to learn to turn at 2M steps, opting to drive slowly straight into the wall at 0.37 m/s. Discovered that the Quadratic Target Velocity reward and Lane Centering reward granted massive positive step-rewards just for existing. The slower it drove, the more steps it stayed alive, accumulating more reward before the crash. 
  - Overhauled `rewards.py`: `progress_reward` is now strictly linear `forward_speed` (Total Reward = Distance Traveled, negating the time exploit).
  - `lateral_error_reward` is now a pure penalty.
  - **Stagnation Bias Fix:** The agent then tried to stand completely still because the `-200` crash penalty wasn't being correctly applied to timeouts. Fixed `termination_penalty` to use IsaacLab's `env.reset_terminated` instead of `env.reset_buf`. 
  - **Progress Weight:** Increased `progress_reward` weight to 5.0x so driving forward into the wall is mathematically more profitable than standing still, bootstrapping movement.
- **D015:** Fixed "Playing Dead Trap". After the distance reward fix, the agent became so terrified of the `-200` crash penalty that it refused to move, choosing to take a `-50` stagnation penalty over a `-200` wall collision. Disabled `termination_penalty` entirely. Since the reward is now strictly physical distance traveled, crashing naturally stops point accumulation, making the crash penalty redundant and actively harmful to exploration.
- **D016:** Restored environment testing GUI (`debug.sh`) and patched `verify_policy.py` to support the new nested `"policy"` observation structure. Hardened `watchdog.py` self-healing sequence to launch autonomous repair agents asynchronously, preventing self-termination. Replaced memory-heavy subagents with lightweight daemon-based Antigravity summons for 250k milestone reporting.
- **D017:** Migrated to SKRL and replaced the internal architecture with a frozen `ResNet-18` backbone caching visual features (524 dims). Restarted training from Step 0.
- **D018:** The new ResNet brain was crashing instantly due to Phase 2 penalties. Re-implemented Phase 1 Curriculum (disabling `lateral_error`, `stationary`, `heading`, `smoothness`, `jerk`) so the fresh visual brain can learn basic forward momentum first.
- **D019-D021:** Subagent attempted to fix the Endless Runner exploit and conservative agent traps by adjusting `termination_penalty` and introducing bounded `tanh` progress rewards.
- **D022 (ROOT CAUSE FIX):** Discovered via track progress telemetry (`Trk%`, `WPΔ`) that the agent was NEVER navigating the track. Even 1000-step episodes showed `WPΔ: 0`. The `progress_reward` based on local body velocity was fundamentally exploitable. Replaced with `waypoint_progress_reward` using TrackManager waypoint index deltas.
- **D023:** Fixed "Warning Track Trap". Decreased boundary penalty from 0.6 to 0.4 so the agent wouldn't receive a massive negative reward just for exploring near the walls.
- **D024:** Fixed "Stationary Trap" & "GoSignal Exploit". Increased stationary penalty to 50.0 and removed the GoSignal safety override which the agent exploited by staring at the wall to trigger a false-positive stop light, gaining immunity to stationary penalties.
- **D025:** Fixed "Blind Training". Discovered `--enable_cameras` was missing from `start_tmux_training.sh`. The agent was training completely blind with a 12-dim input instead of the 524-dim ResNet-18 vision system. Added the flag and completely wiped all old blind checkpoints to start fresh. Added strict RGB-only directive to monitor agent.

## Blockers
- None active. Monitor subagent was killed (API quota exhausted). Needs re-launch manually.

## Recent Decisions (Session 2026-07-31)
- **D026:** Fixed **Issue 49** — Last commit (`dac8074`) re-enabled Phase 2 penalties (`heading=2.0`, `smoothness=2.0`) causing penalty over-saturation on a newborn vision agent. Reverted both to `0.0` to restore Phase 1 curriculum.
- **D027:** Fixed **Issue 50 (ROOT CAUSE)** — New `waypoint_progress_reward` in `dac8074` used a high-water-mark that silently returned `0.0` reward for all short episodes (~100 steps). Agent learned from survival/drive rewards only — zero progress gradient — causing a locked ~100-step plateau at 145k steps with consistent `WPΔ_rew: 0.0`. Reverted to simple per-step forward delta in `rewards.py`. Also bumped `termination_penalty` weight to 20.0 (-1000 crash). WPΔ_rew now confirmed firing at ~0.5–0.8.
- **D028:** Camera pose from `dac8074` confirmed working via debug frames — clear view of yellow centerline and white boundaries at `pos=(-0.5, 0.0, 0.35)` with 15° downward pitch. Kept.
- **D029:** Investigated continuous physics explosions and 'snowplow' steering bugs. Discovered the USD model has perfectly symmetric and un-mirrored joints/knuckles, meaning `invert_right_joint` was mathematically incorrect and actively causing the snowplow. Removed `invert_right_joint`. Fixed physical 'toe-in' under load by setting steering actuator stiffness to `400.0` (with `effort_limit_sim=100.0`) and clamping the joints natively in USD to `+/- 35` degrees to prevent 360-degree glitched spinning. Also lowered CoM to `-0.05` and normalized tire friction to `1.0/0.8` (down from `3.0`) to stop the RC car from violently tipping over under extreme lateral grip.

## Recent Decisions (Session 2026-08-12)
- **D030:** **Stationary Min-Max Trap Verified**: Identified that the RL agent mathematically learned to hold throttle at `0.0` because the stagnation penalty (`-550`) was far cheaper than the crash penalty (`-1100`). Applied a massive `15.0` weight to the stationary penalty in `arcpro_env_cfg.py` to fix this.
- **D031:** **play_skrl.py Crash Fix**: Fixed a bug where omitting `--enable_cameras` in the CLI failed to disable cameras in `play_skrl.py`, causing Isaac Sim to crash when loading the environment.
- **D032:** **Checkpoint Mismatch**: Discovered `train_skrl.py` hardcodes the SKRL run class name as `TelemetryPPO` regardless of whether the agent is actually using the 524-input Vision network. Viewers MUST always use `--enable_cameras` to properly load the main branch checkpoints since they are actually Vision models despite the folder name.
- **D033:** **Deleted git worktree**: Removed the `v1.0-working` worktree. The current `main` branch physics are fully verified with `test_physics_rule_based.py`.

## Recent Decisions (Session 2026-08-14)
- **D034:** **180-Degree USD Flip Bug Resolved**: Fixed a critical physics bug where the `track_dir` math penalized forward movement because the visual nose pointed +X while the physics root pointed -X. Set `b_drive` scale to `+20.0`, flipped distance accumulation, and added `+ pi` to yaw in observations. `WPs_cum` now correctly rockets up positively.
- **D035:** **Clean Slate Training**: Due to heavy waypoint poisoning and stationary trap exploitation in old checkpoints, we purged `logs/ppo_skrl` and `skrl_phase1*.log` and restarted `./start_tmux_training.sh` from **Step 0**.
- **D036:** **Submodule Commit & Push**: Resolved Git detached HEAD conflicts in `policy_stack` submodule by forcing `main` to the newest commits and dropping deleted files. Pushed both the submodule and the root repository to `origin/main`.

## Next Action
- Monitor the fresh training phase (from Step 0) in the `training` tmux session to ensure the agent correctly learns forward momentum without finding new exploits.
