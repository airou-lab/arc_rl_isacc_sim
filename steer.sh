#!/bin/bash
# ARCPro RL - Straight Line Test Viewer
# Launches the open-loop straight-line test in a GUI window for visual inspection of physics.

set -euo pipefail

PROJECT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
ISAACLAB_PATH="$HOME/IsaacLab/isaaclab.sh"

# Ensure GUI can launch
if [ -z "${DISPLAY:-}" ]; then
    export DISPLAY=:0
fi

echo "============================================"
echo "  ARCPro RL - Steering Physics Test"
echo "  Script: play_steer.py"
echo "  Description: Applies 50% throttle and 50% steering."
echo "============================================"
echo ""

exec "$ISAACLAB_PATH" -p "$PROJECT_DIR/arcproLab/scripts/play_steer.py" \
    --num_envs 1 \
    "$@"
