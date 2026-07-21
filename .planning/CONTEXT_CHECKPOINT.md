# ARCPro RL Curriculum - Context Checkpoint

## Current Status
**Phase 1** (Forward Momentum & Basic Driving) is **100% COMPLETE**. The agent successfully discovered the mathematical limits of the simulated chassis, reaching speeds of nearly `0.80 m/s` while surviving up to `250 steps` before hitting the physical limit of the first major curve.

**Phase 2** (Waypoint Tracking & Centerline Alignment) is **ACTIVE**.
- The `logs/` directory was wiped clean and the training was completely restarted from Step 0 with a new brain.
- The `arcpro_env_cfg.py` was updated with the Phase 2 reward structure:
  - `progress_reward` weight was reduced from `50.0` to `10.0` to discourage "suicide drag-racing".
  - `lateral_error_reward` (weight `1.0`) and `heading_alignment_reward` (weight `0.5`) were enabled to strictly enforce lane centering.
  - `jerk_penalty` (weight `1.0`) and `smoothness_reward` (weight `0.5`) were enabled to eliminate jittery steering corrections.

## Training Performance (Phase 2 - 500k Timesteps)
As of the latest check (approx 500k timesteps):
- The agent initially experienced a massive reward drop (down to `-208`) as it was slammed with the new penalties.
- It has since rapidly learned to stay in the center of the lane to avoid penalties, bringing its reward back up to **`-21.2`**.
- It is currently holding a steady speed of **`0.625 m/s`** while attempting to corner smoothly.

## Next Steps for New Agent Session
1. **Monitor Phase 2:** Continue to check `logs/curriculum_phase1.log` to watch the agent's reward break into positive numbers. The watchdog cron job (`task-500` if it survived the clear, or recreate it) should be kept active.
2. **Verify Policy:** The user can run `/home/arika/IsaacLab/isaaclab.sh -p arcproLab/scripts/verify_policy.py` locally to visually inspect the agent's new cornering abilities.
3. **Phase 3 Planning:** Once the agent successfully achieves a high positive reward and survives the entire track without flipping or hitting boundaries, it will be time to plan Phase 3 (possibly obstacle avoidance or dynamic speeds).

## Active Infrastructure
- **tmux session:** `training` (Running `train_policy.py` and `watchdog.py` concurrently).
- **Watchdog:** Monitoring the training and will write to `logs/WATCHDOG_DIAGNOSIS.md` if the training diverges or stagnates.
