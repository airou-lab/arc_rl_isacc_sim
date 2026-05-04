#!/bin/bash
PROJECT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
ISAACLAB_PATH="/home/arika/IsaacLab/isaaclab.sh"
LOG_FILE="$PROJECT_DIR/training_stabilized.log"

echo "Launching stabilized training (8 envs) at $(date)" > "$LOG_FILE"
PYTHONUNBUFFERED=1 $ISAACLAB_PATH -p "$PROJECT_DIR/arcproLab/scripts/train_policy.py" \
    --num_envs 8 \
    --headless \
    --total_timesteps 5000000 \
    --enable_cameras >> "$LOG_FILE" 2>&1 &
echo "Training launched in background with PID $!" >> "$LOG_FILE"
