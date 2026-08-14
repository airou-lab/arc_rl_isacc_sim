Monitor the RL training process by reading the log at `logs/skrl_phase1.log` periodically. The training is running in a tmux session called 'training'. Use the `schedule` tool to set up a timer to check the log.

Your objective is to trigger an analysis and auto-fix every 0.6M (600,000) timesteps, AND immediately notify the parent agent if the strict target (`ep_len_mean >= 600` AND `speed_mps > 0.6`) is met.

**CRITICAL DIRECTIVE (RGB-Only Self Driving):** The ultimate goal of this project is true RGB-only self-driving. You MUST ensure that `--enable_cameras` is always present in the `start_tmux_training.sh` command.

**CRITICAL CONTEXT:** The agent has been exploiting a 'Spinning Top' loophole for the last several runs. We rewrote `stationary_penalty` in `rewards.py` to check actual waypoint progress (not raw velocity). The penalty now checks every 10 steps whether cumulative waypoint progress has advanced ≥5 WPs. If not, it fires -5.0 * weight(10) = -50 per check. Over 401 spinning steps, this yields ~-1,750 stagnation penalty.

**IMPORTANT REWARD HISTORY:** Read `.planning/reward_tuning_history.md` before making ANY changes. Issues 1-68 document the full history of reward shaping fixes. Do NOT revert any of these fixes without understanding why they were made.

Steps:
1. Check `total_timesteps` in the log (iterations × 32 envs).
2. If it has crossed a new 0.6M milestone since your last check:
   - Analyze `ep_rew_mean`, `speed_mps`, AND CRITICALLY `ep_len_mean` and `WPs_cum`.
   - If `WPs_cum` is still 0 after 0.6M steps, the agent is STILL not driving. Research and fix.
   - If training is bad, read `.planning/reward_tuning_history.md` FIRST to avoid repeating old fixes.
   - Apply researched fix to `arcproLab/arcpro_env_cfg.py` or `arcproLab/mdp/rewards.py`.
   - Append the new Issue and Fix to `.planning/reward_tuning_history.md`.
   - Delete checkpoints: `rm -rf logs/ppo_skrl/*`
   - Restart training: `./start_tmux_training.sh`
   - ALWAYS use `send_message` to notify the parent agent with the metrics!
3. If it hasn't crossed the next milestone, just wait.

Start by checking the current step count in `logs/skrl_phase1.log` and estimating when 0.6M will be reached. Schedule your first timer accordingly.
