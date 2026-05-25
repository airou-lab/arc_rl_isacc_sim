#!/bin/bash

# Get the absolute path of the current directory
PROJECT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

# Define the Isaac Lab path
ISAACLAB_PATH="${AARON_WORKSPACE:-$HOME/aaron_workspace}/isaac_setup/IsaacLab/isaaclab.sh"

# FIND LATEST MODEL
# Search for .zip files in logs/ppo, sort by modification time, and take the most recent
LATEST_CHECKPOINT=$(find "$PROJECT_DIR/logs/ppo" -name "*.zip" -printf "%T@ %p\n" | sort -n | tail -1 | cut -d' ' -f2-)

if [ -z "$LATEST_CHECKPOINT" ]; then
    echo "Error: No model checkpoints found in logs/ppo"
    exit 1
fi

echo "--------------------------------------------------"
echo "Launching F1Tenth Visual Verification (Latest)"
echo "Project Dir: $PROJECT_DIR"
echo "Isaac Lab:   $ISAACLAB_PATH"
echo "Checkpoint:  $LATEST_CHECKPOINT"
echo "--------------------------------------------------"

# Run the verification script with GUI enabled
$ISAACLAB_PATH -p "$PROJECT_DIR/arcproLab/scripts/verify_live.py" \
    --checkpoint "$LATEST_CHECKPOINT" \
    --num_envs 1
