Monitor the RL training process by reading the TensorBoard logs (or `logs/skrl_phase1.log`) periodically. Use the `schedule` tool to set up a recurring cron job (e.g. `*/5 * * * *`) to check the log.

Your objective is to trigger an analysis and auto-fix every 0.5M (500,000) timesteps, AND immediately notify the parent agent if the strict target (`ep_len_mean >= 600` AND `speed_mps > 0.6`) is met.

**CRITICAL DIRECTIVE (RGB-Only Self Driving):** The ultimate goal of this project is true RGB-only self-driving. You MUST ensure that `--enable_cameras` is always present in the `start_tmux_training.sh` command. If the agent trains blindly (without cameras) or relies on unmasked mathematical telemetry (like lateral error or heading) for its actions, the run is considered invalid.

1. Check `total_timesteps` in the log.
2. If it has crossed a new 500k milestone (e.g. 500k, 1M, 1.5M, 2M, etc.) since your last check:
   - Analyze `ep_rew_mean`, `speed_mps`, AND CRITICALLY `ep_len_mean`.
   - IMPORTANT: Even if the reward is positive, if `ep_len_mean` is consistently low (e.g., < 200 steps), it means the agent is exploiting the reward system and crashing early! This is a BAD state.
   - If training is bad (suicide, low ep_len_mean, stuck, or massive negative reward), do NOT just guess a fix.
   - First, use the `search_web` tool or invoke a `research` subagent to look up best practices for RL reward shaping related to the specific failure mode in IsaacLab/PyTorch.
   - Next, read `.planning/reward_tuning_history.md` to ensure your proposed fix hasn't already been tried and failed.
   - Apply your researched fix to `arcproLab/arcpro_env_cfg.py`.
   - Append the new Issue and Fix (along with the research theory) to the bottom of `.planning/reward_tuning_history.md`.
   - IMPORTANT: Because the environment dynamics or rewards changed, you MUST delete the existing checkpoints so it starts from scratch. Run `rm -rf logs/ppo_skrl/*` BEFORE restarting.
   - Restart the training using `./start_tmux_training.sh`.
   - ALWAYS use `send_message` to notify the parent agent whenever you hit a milestone, whether you had to fix it or not! Include the metrics in your message.
3. If it hasn't crossed the next milestone, just wait for the next cron trigger.
