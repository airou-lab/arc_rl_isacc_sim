# Reward Tuning History
This file tracks the history of issues encountered during RL training and the reward tuning fixes applied.
Always read this file before modifying the reward function to ensure you do not repeat past failed configurations.

## History

**Issue 1**: The agent found a "suicide loophole" where it would sprint forward to earn `progress_reward` and intentionally crash into the wall early (`ep_len_mean` < 200) because the crash penalty was `0.0`.
**Fix 1**: We iteratively increased `termination_penalty` from `0.0` -> `1.0` -> `50.0` -> `500.0` -> `2000.0`.
**Result**: Success! The agent stopped crashing early and survived for up to 450 steps per episode.

**Issue 2**: The agent was surviving but earning massive step penalties (`ep_rew_mean` ~ -2040). It was violently wiggling its steering to avoid the boundaries, triggering huge `jerk_penalty` and `lateral_error` penalties on the straightaways instead of learning smooth cornering.
**Fix 2**: We increased `lateral_error` weight from `0.5` to `2.0` and decreased `jerk_penalty` weight from `0.5` to `0.1`.
**Result**: Failure! The increased `lateral_error` made surviving and wiggling mathematically worse than crashing. The agent reverted to the suicide loophole (`ep_len_mean` dropped to 121, speed spiked to 0.64 m/s).

**Issue 3**: Agent regressed to suicide loophole due to high `lateral_error` penalty.
**Fix 3**: Reverted `lateral_error` from `2.0` back down to `0.5`. Doubled `progress_reward` from `25.0` to `50.0` to massively over-incentivize surviving and moving forward to offset any remaining step penalties.
**Result**: Currently testing in the active run.
