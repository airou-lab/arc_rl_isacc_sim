#!/bin/bash
# ARCPro RL - GUI Debug Viewer (Environment Only)
# Optimized for pure coordinate and termination verification.

PROJECT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
ISAACLAB_PATH="$HOME/IsaacLab/isaaclab.sh"

# Default to 1 environment for GUI debugging
NUM_ENVS=1

# Ensure GUI can launch
if [ -z "$DISPLAY" ]; then
    export DISPLAY=:0
fi

echo "Launching Environment Verification (Forward Drive Only)..."
echo "Command: $ISAACLAB_PATH -p arcproLab/scripts/verify_policy.py --num_envs $NUM_ENVS --debug --enable_cameras"

$ISAACLAB_PATH -p "$PROJECT_DIR/arcproLab/scripts/verify_policy.py" \
    --num_envs $NUM_ENVS \
    --debug \
    --enable_cameras \
    "$@"
