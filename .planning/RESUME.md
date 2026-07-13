# ARCPro RL Training - Resume Context
**Date:** 2026-07-13

## Current Status
- Training an RL agent using IsaacLab for an autonomous driving task.
- We have been dealing with a failure mode where the agent wiggles and crashes intentionally (suicide loophole) to avoid continuous `lateral_error` penalties on straightaways.
- The reward tuning history is fully documented in `.planning/reward_tuning_history.md`.
- There is an autonomous subagent designed to run in the background, monitoring the log (`logs/curriculum_phase1.log`), researching RL reward theory, and auto-tuning the reward weights every 250k steps.

## Most Recent Changes
- In the latest session, the agent was crashing early at step 121 (speed 0.64).
- The monitor subagent autonomously reduced `lateral_error` from `2.0` to `0.5` and increased `progress_reward` from `25.0` to `50.0`.
- The training was restarted and is currently running via `./start_tmux_training.sh` inside a `tmux` session.

## Next Steps for the New Agent
1. **Understand Context:** Read `.planning/reward_tuning_history.md` to understand all past reward tuning attempts.
2. **Revive the Monitor:** Read `.planning/monitor_agent_prompt.md` and use the prompt to spawn a new `self` subagent to resume the background monitoring and auto-tuning.
3. **Check Progress:** Check the `logs/curriculum_phase1.log` to see how the current run (with `lateral_error: 0.5` and `progress_reward: 50.0`) is progressing.
