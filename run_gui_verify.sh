#!/bin/bash

# Get the absolute path of the current directory
PROJECT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

# Define the Isaac Lab path
ISAACLAB_PATH="/home/arika/IsaacLab/isaaclab.sh"

# Use the latest 140k model checkpoint
CHECKPOINT="$PROJECT_DIR/logs/ppo/20260411-160326/model_140000_steps.zip"

echo "--------------------------------------------------"
echo "Launching F1Tenth Visual Verification (v1.2)"
echo "Project Dir: $PROJECT_DIR"
echo "Isaac Lab:   $ISAACLAB_PATH"
echo "Checkpoint:  $CHECKPOINT"
echo "--------------------------------------------------"

# Run the verification script with GUI enabled
# Using verify_live.py because it correctly loads VecNormalize stats
$ISAACLAB_PATH -p "$PROJECT_DIR/arcproLab/scripts/verify_live.py" \
    --checkpoint "$CHECKPOINT" \
    --num_envs 1
