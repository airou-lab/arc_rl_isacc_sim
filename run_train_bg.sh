#!/bin/bash
# Start background training with 8 environments (VRAM Optimized)
LOG_FILE="logs/production_mastery_8_relaunch.log"

nohup /home/arika/IsaacLab/isaaclab.sh -p arcproLab/scripts/train_policy.py \
    --num_envs 8 \
    --headless \
    --total_timesteps 5000000 \
    --enable_cameras \
    > "$LOG_FILE" 2>&1 &

echo "Training started in background (8 envs). Log: $LOG_FILE"
echo "PID: $!"
