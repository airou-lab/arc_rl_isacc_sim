# RESUME — Context for Next Agent Session
**Last Updated:** 2026-08-14T00:55 CDT

---

## Current Status
- **Training is RUNNING in tmux (`training` session)**. We successfully restarted training from **Step 0** with `./start_tmux_training.sh` after purging all old logs and checkpoints.
- **Major Milestone Achieved**: The math bugs causing massive negative waypoint progress have been completely resolved! The car successfully drives straight and is rewarded positively.
- **Git Push**: All unresolved conflicts and changes in the `policy_stack` submodule have been merged, committed, and pushed. The root repository has also been committed and pushed to `main`.
- We are currently monitoring this fresh training phase to ensure the agent learns to drive around the entire track correctly.

## What This Session Accomplished

### 1. Training Restart from Scratch (Clean Logs)
- **Root Cause:** The agent's old checkpoints were heavily poisoned by the previous bugs (waypoint math bugs and the stationary min-max trap). Even resuming from a clean baseline (`agent_14450.pt`) showed heavy exploration drift.
- **Fix:** We completely deleted the `logs/ppo_skrl` directory and all `skrl_phase1*.log` files to force the training script to start completely from scratch (Step 0) instead of auto-resuming.
- **Result:** Training is now fresh and unpoisoned. (Note: `WPs_cum` occasionally hitting `-1` early in training is normal physics behavior from the car bouncing/sliding backward off walls, not a bug).

### 2. USD Mesh 180-Degree Flip Bug (RESOLVED)
- **Root Cause:** The F1Tenth USD 3D model's physics root X-axis points towards its REAR WING, completely opposite to the visual mesh's nose. Because the root X-axis points backward, driving nose-first (positive scale) causes the physics engine to report the car is moving in its local `-X` direction. This flipped the `yaw` vector 180 degrees, tricking the `track_dir` math into penalizing forward movement and rewarding backward movement.
- **Fix:** We permanently set `scale=20.0` (positive) in `arcpro_env_cfg.py` so the wheels drive visually forward. We then added a `+ pi` (180 degree) rotation to the `yaw` extraction in `arcproLab/mdp/observations.py` to artificially align the physics root with the visual nose. We also fixed the `distance` accumulator to subtract the local X velocity (since it's flipped).
- **Result:** `WPs_cum` now correctly rockets up into the positive thousands when driving forward!

### 3. SKRL Memory Size Overflow (RESOLVED)
- **Root Cause:** OOM errors during training due to extreme replay buffer sizes.
- **Fix:** We hardcoded `memory_size=1024` in `train_skrl.py` to prevent memory blowouts on the agent parameters.

## Current Reward Configuration (Phase 1: Straight Driving)
```python
survival_bonus = 0.0        # Disabled (spinning top farming)
termination_penalty = 10.0  # -500 on crash
progress_reward = 200.0     # Massive incentive for waypoints
action_steer_penalty = 0.0  
action_drive_reward = 0.5   
lateral_error = 0.0         # Disabled (Phase 1)
stationary = 100.0          # Massive penalty for standing still
heading = 10.0              # Conditional survival bonus
smoothness = 2.0            # Action rate smoothness
jerk = 0.0                  # Disabled (Phase 1)
boundary = 0.4              # -40 per step in warning track
```

## Active Issues / Blockers
1. **None!** We are currently waiting on the training session to complete/progress to see if the agent successfully learns the full track without finding a new exploit.

## Key Files Modified This Session
| File | Changes |
|------|---------|
| `arcproLab/arcpro_env_cfg.py` | Restored `b_drive` scale to positive 20.0 |
| `arcproLab/mdp/observations.py` | Added 180-degree `yaw` fix, flipped local X distance accumulator |
| `arcproLab/scripts/train_skrl.py` | Set `memory_size=1024` to fix OOM |
| `.planning/reward_tuning_history.md` | Documented the USD flip bug (Fix 31 amended) |

## Next Steps
1. **Monitor Training:** Run `tmux attach -t training` periodically to check progress.
2. **Verify Learning:** Once training yields a good checkpoint, run `isaaclab.sh -p arcproLab/scripts/play_skrl.py` to ensure the agent handles turns correctly.
3. **Phase 2 (Turns & Overrides):** If Phase 1 succeeds, we will re-enable `lateral_error` and begin training the agent to take racing lines instead of just driving straight.
