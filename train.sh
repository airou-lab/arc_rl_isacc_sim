#!/bin/bash

# Get the absolute path of the current directory
PROJECT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

# Define the Isaac Lab path
ISAACLAB_PATH="/home/arika/IsaacLab/isaaclab.sh"

echo "--------------------------------------------------"
echo "Starting F1Tenth RL Training"
echo "Project Dir: $PROJECT_DIR"
echo "--------------------------------------------------"

# Run training
# --headless: Faster training without GUI
# --num_envs: Number of parallel simulation environments
# --enable_cameras: Required for vision-based policies
$ISAACLAB_PATH -p "$PROJECT_DIR/arcproLab/scripts/train_policy.py" --num_envs 1 --headless --total_timesteps 1000000 --enable_cameras
