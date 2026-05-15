#!/bin/bash

# Get the absolute path of the current directory
PROJECT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

# Define the Isaac Lab path
ISAACLAB_PATH="/home/arika/IsaacLab/isaaclab.sh"

# Default checkpoint
CHECKPOINT_ARG=""
DECLARE_REMAINING=()

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --checkpoint)
            CHECKPOINT_ARG="--checkpoint $2"
            shift 2
            ;;
        *)
            DECLARE_REMAINING+=("$1")
            shift
            ;;
    esac
done

# If no checkpoint provided, find the latest .zip that is NOT a backup
if [ -z "$CHECKPOINT_ARG" ]; then
    LATEST_CHECKPOINT=$(find "$PROJECT_DIR/logs/ppo" -name "*.zip" -printf "%T@ %p\n" | sort -n | tail -1 | cut -d' ' -f2-)
    if [ -n "$LATEST_CHECKPOINT" ]; then
        echo "Checkpoint (Auto-detected): $LATEST_CHECKPOINT"
        CHECKPOINT_ARG="--checkpoint $LATEST_CHECKPOINT"
    fi
else
    echo "Checkpoint (Specified): ${CHECKPOINT_ARG#--checkpoint }"
fi

echo "--------------------------------------------------"

# Ensure DISPLAY is set for GUI
export DISPLAY=:0

# Final Command execution
echo "Executing: $ISAACLAB_PATH -p $PROJECT_DIR/arcproLab/scripts/verify_policy.py $CHECKPOINT_ARG ${DECLARE_REMAINING[@]}"
$ISAACLAB_PATH -p "$PROJECT_DIR/arcproLab/scripts/verify_policy.py" \
    $CHECKPOINT_ARG \
    "${DECLARE_REMAINING[@]}"
