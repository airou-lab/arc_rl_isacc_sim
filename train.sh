#!/bin/bash

# Get the absolute path of the current directory
PROJECT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

# Define the Isaac Lab path
ISAACLAB_PATH="${AARON_WORKSPACE:-$HOME/aaron_workspace}/isaac_setup/IsaacLab/isaaclab.sh"

echo "--------------------------------------------------"
echo "Starting F1Tenth RL Training"
echo "Project Dir: $PROJECT_DIR"
echo "--------------------------------------------------"

# Default values
NUM_ENVS=1
HEADLESS=""
CHECKPOINT=""

# Parse arguments
while [[ "$#" -gt 0 ]]; do
    case $1 in
        --num_envs) NUM_ENVS="$2"; shift ;;
        --headless) HEADLESS="--headless" ;;
        --checkpoint) CHECKPOINT="--checkpoint $2"; shift ;;
        *) echo "Unknown parameter passed: $1"; exit 1 ;;
    esac
    shift
done

# Run training
PYTHONUNBUFFERED=1 $ISAACLAB_PATH -p "$PROJECT_DIR/arcproLab/scripts/train_policy.py" \
    --num_envs $NUM_ENVS \
    $HEADLESS \
    --total_timesteps 1000000 \
    --enable_cameras \
    $CHECKPOINT
