#!/bin/bash
PROJECT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
ISAACLAB_PATH="/home/arika/IsaacLab/isaaclab.sh"
LOG_FILE="$PROJECT_DIR/training_production_5kg_fresh.log"

echo "Launching 5kg REALISTIC fresh training at $(date)" > "$LOG_FILE"
# Clean sweep of any ghost processes
ps aux | grep -E "IsaacLab|kit|python" | grep -v grep | awk '{print $2}' | xargs -r kill -9
sleep 2

nohup env PYTHONUNBUFFERED=1 $ISAACLAB_PATH -p "$PROJECT_DIR/arcproLab/scripts/train_policy.py" \
    --num_envs 32 \
    --headless \
    --total_timesteps 10000000 \
    --enable_cameras >> "$LOG_FILE" 2>&1 &

echo "Production training (5kg) launched in background with PID $!" >> "$LOG_FILE"
