#!/bin/bash
# Cold-start training: 16 envs, batch_size 64.
# (VRAM is too tight for batch_size 96/128 with ResNet + 16 envs. Dropped back to 64).
# PPO stability fixes: n_epochs 4 (down from 10).
# Reward shaping fixes: progress_reward replaces flat speed_reward to punish creeping.
# Launches inside a detached tmux session so the run shows up in `tmux ls`
# and can be attached with `tmux attach -t <session>`.
TS="$(date +%Y%m%d-%H%M%S)"
LOG_FILE="logs/cold_start_16env_fix_b64_${TS}.log"
SESSION="train-${TS}"

REPO_DIR="$(cd "$(dirname "$0")" && pwd)"

CMD="cd '$REPO_DIR' && /home/arika/IsaacLab/isaaclab.sh -p arcproLab/scripts/train_policy.py \
    --num_envs 16 \
    --batch_size 64 \
    --headless \
    --total_timesteps 5000000 \
    --enable_cameras \
    2>&1 | tee '$LOG_FILE'"

tmux new-session -d -s "$SESSION" "$CMD"

echo "Training started in tmux session: $SESSION (16 envs, batch_size 64, cold start)"
echo "Log: $LOG_FILE"
echo "Attach: tmux attach -t $SESSION"
