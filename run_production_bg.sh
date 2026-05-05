#!/bin/bash
PROJECT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
ISAACLAB_PATH="/home/arika/IsaacLab/isaaclab.sh"
LOG_FILE="$PROJECT_DIR/logs/training_production_recovery.log"
CHECKPOINT="$PROJECT_DIR/logs/ppo/20260502-170607/model_1000000_steps.zip"

echo "Launching production recovery training from 1M checkpoint at $(date)" > "$LOG_FILE"
# Kill any leftover processes first
ps aux | grep -E "IsaacLab|kit|python" | grep -v grep | awk '{print $2}' | xargs -r kill -9
sleep 2

PYTHONUNBUFFERED=1 $ISAACLAB_PATH -p "$PROJECT_DIR/arcproLab/scripts/train_policy.py" \
    --num_envs 32 \
    --headless \
    --total_timesteps 10000000 \
    --enable_cameras \
    --checkpoint "$CHECKPOINT" >> "$LOG_FILE" 2>&1 &
echo "Recovery training launched in background with PID $!" >> "$LOG_FILE"
