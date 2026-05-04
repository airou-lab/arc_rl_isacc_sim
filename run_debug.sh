#!/bin/bash

# Get the absolute path of the current directory
PROJECT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

# Define the Isaac Lab path
ISAACLAB_PATH="/home/arika/IsaacLab/isaaclab.sh"

# FIND LATEST MODEL
LATEST_CHECKPOINT=$(find "$PROJECT_DIR/logs/ppo" -name "*.zip" -printf "%T@ %p\n" | sort -n | tail -1 | cut -d' ' -f2-)

if [ -z "$LATEST_CHECKPOINT" ]; then
    echo "Warning: No model checkpoints found in logs/ppo. Launching for visual verification only."
    CHECKPOINT_ARG=""
else
    echo "Checkpoint:  $LATEST_CHECKPOINT"
    CHECKPOINT_ARG="--checkpoint $LATEST_CHECKPOINT"
fi

echo "--------------------------------------------------"
echo "Running GUI in DEBUG mode..."
echo "--------------------------------------------------"

# Ensure DISPLAY is set for GUI
export DISPLAY=:0

# Run the verification script with GUI enabled AND debug flag
$ISAACLAB_PATH -p "$PROJECT_DIR/arcproLab/scripts/verify_policy.py" \
    $CHECKPOINT_ARG \
    --num_envs 1 \
    --debug