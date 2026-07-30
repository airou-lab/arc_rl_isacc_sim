# Context Resume Guide

## Current Status
- **Project:** ARCPro Reinforcement Learning for autonomous driving (IsaacLab / SKRL).
- **Goal:** True RGB-only self-driving on the `openStreetUSD` track using a frozen `ResNet-18` backbone.
- **State:** The agent is actively training in the background. It is currently in a "Phase 1 Curriculum" (strict lane-centering penalties are temporarily disabled so it can safely learn to corner without penalty-induced fear).
- **Recent Breakthrough:** The agent reached 639 steps (successfully cornering) but suffered catastrophic forgetting due to a tiny PPO batch size. We just fixed this by uncorking the PPO memory buffer, stabilizing its learning gradients.

## Most Recent Changes
We resolved several critical exploits/bugs:
1. **Issue 40 (Suicide Loophole):** Lowered `stationary` penalty from 200 to 15. The agent was intentionally crashing to escape the massive per-step penalty of slowing down.
2. **Issue 41 (Phase 2 Penalty Over-Saturation):** Disabled `lateral_error` and `action_steer_penalty` so the agent is allowed to wobble while it learns basic visual cornering.
3. **Issue 42 (Catastrophic Forgetting):** Modified `train_skrl.py` to increase `rollouts` and `memory_size` to 1024, yielding a massive, stable batch size of 2,560 for PPO.

## Active Background Tasks / Subagents
1. **Tmux Session:** There is a detached tmux session named `training` running the Isaac Sim RL environment (`./start_tmux_training.sh`).
2. **Subagent:** The `strict_monitor_agent` is running in the background (via cron) to monitor the logs every 15 minutes. It will notify you automatically when the agent hits the 500k or 1 Million step milestones.

## Next Steps for Incoming Agent
1. Wait for the user's prompt or the subagent's milestone report.
2. Check `logs/skrl_phase1.log` (via `tail -n 30`) to observe the latest `ep_len_mean` and `speed_mps`.
3. If the agent successfully masters cornering (consistently surviving > 1000 steps), the next objective will be to re-enable the Phase 2 precision penalties (`lateral_error` and `action_steer_penalty`) in `arcpro_env_cfg.py` to force it to perfectly center itself in the lane.
