# RESUME — Context for Next Agent Session
**Last Updated:** 2026-08-09T19:55 CDT (Session: `4f10ca7c`)

---

## Current Status
- **Training is STOPPED**. The previous tmux session was killed to free up VRAM for the new local LLM pipeline.
- **The agent was STILL stuck spinning in place** at ~121k steps (Spd: 0.01, WPs_cum: 0, Rew: -526).
- **Architecture Shift:** We have built a local, autonomous self-healing pipeline using Ollama (Qwen 2.5 Coder 7B) and Aider to avoid API token costs.
- The `WarmupActionWrapper` in `train_skrl.py` is configured to hold the car still for 10 steps (throttle `-1.0` = Stop) to let the physics engine settle.

## What This Session Accomplished

### 1. Steering Physics Fix (RESOLVED)
- **Root Cause:** `invert_right_joint=True` was wrong — USD audit proved both `Joint_Steer_L` and `Joint_Steer_R` are symmetric. The flag was causing "snowplow" (wheels facing inward).
- **Fix:** Removed `invert_right_joint` from `arcpro_env_cfg.py`. Added `_apply_steering_limits()` to `spawner.py` setting ±35° hard joint limits.

### 2. Ackermann Math Overflow Fix (RESOLVED)
- **Root Cause:** The NN occasionally output unbounded actions (e.g., `-3.59`). When `steering_angle > π/2`, `tan()` flips sign, corrupting the Ackermann geometry and causing wheels to face opposite directions.
- **Fix:** Added `torch.clamp(actions, -1.0, 1.0)` in both `AckermannSteeringAction` and `GroupedJointVelocityAction` in `actions.py` before any math.

### 3. Spawn Direction Fix (RESOLVED)
- **Root Cause:** The last commit (`e49c5fa`) had spawn rotation facing North, but the track flows South at the spawn point.
- **Fix:** Changed `rot` in `arcpro_env_cfg.py` init_state to `(0.7071, 0.0, 0.0, -0.7071)` (Face South).

### 4. The Spinning Top Exploit (ACTIVE — IN PROGRESS)
The RL agent discovered it could spin in tight circles near spawn to avoid crashing. This session fought 8+ iterations of this exploit:

| Issue | Exploit | Fix | Result |
|-------|---------|-----|--------|
| 61 | Suicide (79 steps) — stationary penalty too high vs survival | Reduced `stationary` 15→5 | Stopped suicide |
| 62 | Conservative crawl (0.09 m/s) — crash penalty too scary | Disabled `action_steer_penalty`, reduced `termination_penalty` 10→2 | Stopped crawling |
| 63 | Lazy crash (59 steps) — crash too cheap | Restored `termination_penalty` to 10, `boundary` to 0.6, `stationary` to 10 | Stopped lazy crash |
| 64 | Spinning Top (241 steps) — farming survival bonus while spinning | Disabled `survival_bonus` (weight 0.0) | Stopped survival farming |
| 65 | Spin-out backward (64 steps, WPs -7) — no incentive to face forward | Increased `heading` reward 2→10 | Stopped spin-outs |
| 66 | Stationary Min-Max (0.06 m/s) — heading reward farmed while still | Made `heading_alignment_reward = speed * cos(head_err)` | Stopped still farming |
| 67 | Unkillable Spinning Top (401 steps) — spinning evades stationary penalty | Restored `action_steer_penalty` to -0.5 | Partially helped |
| 68 | Gas Pedal Exploit — farming `action_drive_reward` while spinning | Disabled `action_drive_reward` (weight 0.0) | Partially helped |

**Physics-level fix applied:** Set `max_angular_velocity=5.0` rad/s in `arcpro_robot_cfg.py` (was 1000.0). Didn't fully solve it — agent just spins slower.

**Reward-level fix applied (current):** Completely rewrote `stationary_penalty` in `rewards.py`:
- Old: Checked `speed < 0.2 m/s` (gameable — spinning generates 0.25 m/s centripetal velocity)
- New: Checks **actual waypoint progress** every 10 steps. If `cumulative_wp_index` hasn't advanced ≥5 waypoints, fires `-5.0 * weight(10) = -50/check`.
- Over 401 spinning steps: ~35 checks × -50 = **-1,750** stagnation penalty. Makes spinning catastrophically worse than crashing (-500).
- **Bug found & fixed:** The original threshold of `< 1.0` was too low — TrackManager nearest-waypoint jitter (±1-2 WPs from physics float drift) fooled the penalty. Raised to `< 5.0`.

| 69 | Stationary Min-Max (0.00 m/s) — agent stands perfectly still because `-558` from stagnation is better than `-1100` from crashing | Increased `stationary` penalty significantly to heavily punish standing still | Pending training |

### 5. Current Reward Configuration
```python
survival_bonus = 0.0        # Disabled (spinning top farming)
termination_penalty = 10.0  # -500 on crash
progress_reward = 50.0      # tanh-bounded waypoint progress
action_steer_penalty = -0.5 # Penalizes holding wheel at full lock
action_drive_reward = 0.5   # 
lateral_error = 0.0         # Disabled (Phase 1)
stationary = 15.0           # Waypoint-progress-based, massive penalty for standing still
heading = 2.0              # conditional survival bonus
smoothness = 2.0            # Action rate smoothness
jerk = 0.0                  # Disabled (Phase 1)
boundary = 0.4              # -40 per step in warning track
```

## Active Issues / Blockers
1. **Agent still spinning at step 30k** (Rew: -862, Spd: 0.00, WPs: 0). The harsher penalty IS firing (reward dropped from -551 to -862), but PPO hasn't broken out of the local minimum yet. May need more training time, or a fundamentally different approach (e.g., curriculum with forced forward velocity).
2. **`reward_tuning_history.md` only goes up to Issue 60** in the file. Issues 61-68 were documented by the monitor subagent but may have been lost when it hit rate limits. The next agent should verify and append if missing.

## Key Files Modified This Session
| File | Changes |
|------|---------|
| `arcproLab/arcpro_env_cfg.py` | Removed `invert_right_joint`, spawn rot→South, reward weights tuned (see table above) |
| `arcproLab/arcpro_robot_cfg.py` | `max_angular_velocity` 1000→5.0 |
| `arcproLab/mdp/actions.py` | Added `torch.clamp` to both action classes, debug prints |
| `arcproLab/mdp/rewards.py` | `heading_alignment_reward` now `speed*cos(head_err)`, `stationary_penalty` rewritten to use waypoint progress |
| `arcproLab/mdp/spawner.py` | Added `_apply_steering_limits()` (±35° hard limits) |
| `arcproLab/mdp/observations.py` | Minor edits from monitor subagent |
| `.planning/reward_tuning_history.md` | Issues 61-68 added (verify they're all there) |

## Next Steps
1. **Start the Self-Healing Pipeline:** Run `./train_loop_aider.sh`. This script will launch Isaac Lab. If the agent crashes or the training stagnates (non-zero exit code), it will kill Isaac Lab, wake up Qwen via Ollama, use Aider to fix the python files, update this `.planning` directory, and restart training.
2. **Monitor Qwen's Actions:** Qwen is instructed to write a summary of its fixes to `.planning/QWEN_UPDATE.md`. The main Antigravity agent will read this file to report status to the user.
3. **If agent STILL spinning:** We reverted the curriculum fix (forced forward velocity) to let the physics engine settle properly for the first 10 steps. Qwen will need to solve the spinning top exploit through reward tuning, hyperparameter adjustments, or trying the curriculum fix itself.
4. **Git:** All changes are uncommitted. Consider committing once the spinning exploit is resolved.

## Running Background Processes
- **None.** The previous tmux session and Antigravity monitor subagents have been stopped.
- The user is expected to launch `./start_tmux_training.sh` manually to resume training and unlearn the Stationary Exploit.

## Script & CLI Fixes
- `play_skrl.py` fixed: Now explicitly forces `env_cfg.enable_cameras = False` if `--enable_cameras` is omitted from the CLI, preventing Isaac Sim from crashing on missing camera configs.
- Discovered `train_skrl.py` hardcodes the `TelemetryPPO` class name, causing 524-input Vision checkpoints to save into `_TelemetryPPO` folders. When using the viewer, always pass `--enable_cameras` to properly load the 524-input checkpoints!
- Restored `test_physics_rule_based.py` for physics engine verification without RL agent noise.

## Important Tags/Commits
- `v1.0-working` (commit `e49c5fa`): Last known working physics baseline.
- Current HEAD: Uncommitted changes on top of `e49c5fa` containing critical physics fixes (Ackermann math overflow, invert joint fix, spawn direction fix) which must NOT be lost via a hard reset.
