#!/bin/bash

# Find the latest checkpoint automatically
LATEST_CHECKPOINT=$(find logs/ppo/ -name "*.zip" -printf "%T+ %p\n" | sort -r | head -n 1 | awk '{print $2}')

if [ -z "$LATEST_CHECKPOINT" ]; then
    echo "No checkpoint found in logs/ppo/"
    exit 1
fi

echo "Launching verification with latest checkpoint: $LATEST_CHECKPOINT"

# Launch Isaac Lab verification
/home/arika/IsaacLab/isaaclab.sh -p arcproLab/scripts/verify_policy.py \
    --checkpoint "$LATEST_CHECKPOINT" \
    --num_envs 1 \
    "$@"
