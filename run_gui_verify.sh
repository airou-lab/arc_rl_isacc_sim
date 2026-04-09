#!/bin/bash

# Get the absolute path of the current directory
PROJECT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

# Define the Isaac Lab path (using the known path from context)
ISAACLAB_PATH="/home/arika/IsaacLab/isaaclab.sh"

echo "--------------------------------------------------"
echo "Launching F1Tenth Visual Verification"
echo "Project Dir: $PROJECT_DIR"
echo "Isaac Lab:   $ISAACLAB_PATH"
echo "--------------------------------------------------"

# Run the verification script with GUI enabled
# --enable_cameras: Required for the ResNet18 policy to 'see'
# --num_envs 1: Single environment for focused inspection
$ISAACLAB_PATH -p "$PROJECT_DIR/arcproLab/scripts/verify_policy.py" --num_envs 1 --enable_cameras --max_steps 1000
