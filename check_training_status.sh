#!/bin/bash
# Script to quickly check training status and average episode length

# 1. Check if the training process is currently running
if ! pgrep -f "train_policy.py" > /dev/null; then
    # No training session active, do nothing
    exit 0
fi

# 2. Find the most recently modified log file in the logs/ directory
LATEST_LOG=$(ls -t logs/*.log 2>/dev/null | head -n 1)

if [ -z "$LATEST_LOG" ]; then
    exit 0
fi

# 3. Extract the latest metrics
EP_LEN=$(grep "ep_len_mean" "$LATEST_LOG" | tail -n 1 | awk '{print $4}')
SPEED=$(grep "speed_mps" "$LATEST_LOG" | tail -n 1 | awk '{print $4}')
TIMESTEPS=$(grep "total_timesteps" "$LATEST_LOG" | tail -n 1 | awk '{print $4}')

# 4. Print the output if metrics were found
if [ ! -z "$EP_LEN" ]; then
    echo "Training Active"
    echo "---------------------------"
    echo "Log File   : $LATEST_LOG"
    echo "Timesteps  : $TIMESTEPS"
    echo "Avg Ep Len : $EP_LEN steps"
    echo "Avg Speed  : $SPEED m/s"
fi
