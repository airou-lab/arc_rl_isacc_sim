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
echo "  Tip: defaults to latest clean checkpoint (best_agent.pt)"
echo "  Tip: add --checkpoint pre_reroute to test centered lane-following & intersection turn"
echo "  Tip: add --sector 1..5 to force specific sector (1=Start, 2=C1, 3=Leg2, 4=Inter, 5=North)"
echo "  Tip: add --checkpoint champion to test legacy champion checkpoint"
echo "============================================"
echo ""

exec "$ISAACLAB_PATH" -p "$PROJECT_DIR/arcproLab/scripts/play_skrl.py" \
    --num_envs 1 \
    --enable_cameras \
    "$@"
