#!/bin/bash

# Get the absolute path of the current directory
PROJECT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

# Define the Isaac Lab path (using the known path from context)
ISAACLAB_PATH="${AARON_WORKSPACE:-$HOME/aaron_workspace}/isaac_setup/IsaacLab/isaaclab.sh"

echo "--------------------------------------------------"
echo "Launching F1Tenth METRIC (TRUE PHYSICS) Verification"
echo "Project Dir: $PROJECT_DIR"
echo "Isaac Lab:   $ISAACLAB_PATH"
echo "--------------------------------------------------"

# Run the verification script with GUI enabled
$ISAACLAB_PATH -p "$PROJECT_DIR/arcproLab/scripts/verify_metric.py" --num_envs 1 --headless --enable_cameras
