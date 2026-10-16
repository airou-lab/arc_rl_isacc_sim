#!/bin/bash
# ARCPro RL - Hard-Coded Expert Intersection Viewer
# Launches Isaac Sim GUI to visually verify the car navigating Corner 2 (Intersection)
# using the closed-loop Stanley expert controller with visual debug spheres.

set -euo pipefail

PROJECT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

if [ -z "${DISPLAY:-}" ]; then
    export DISPLAY=:0
fi

echo "============================================"
echo "  ARCPro RL - Hard-Coded Expert Intersection Viewer"
echo "  Sector: 4 (Intersection Approach: X=-6.0, Y=-7.12, East)"
echo "  Controller: Closed-Loop Stanley Expert (tests/test_manual_left_turn.py)"
echo "  Turn: 90-degree left turn into Northbound straight"
echo "============================================"
echo ""

exec "$PROJECT_DIR/debug.sh" --sector 4 --expert --debug "$@"
