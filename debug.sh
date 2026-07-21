#!/bin/bash
# ARCPro RL - GUI Debug Viewer
# Launches the trained SKRL policy in a GUI window for visual inspection.
# play_skrl.py auto-detects the latest checkpoint. Override with --checkpoint.

set -euo pipefail

PROJECT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
ISAACLAB_PATH="$HOME/IsaacLab/isaaclab.sh"

# Ensure GUI can launch
if [ -z "${DISPLAY:-}" ]; then
    export DISPLAY=:0
fi

echo "============================================"
echo "  ARCPro RL - GUI Debug Viewer"
echo "  Script: play_skrl.py"
echo "  Tip: add --checkpoint /path/to/agent.pt to override auto-detection"
echo "============================================"
echo ""

exec "$ISAACLAB_PATH" -p "$PROJECT_DIR/arcproLab/scripts/play_skrl.py" \
    --num_envs 1 \
    "$@"
