#!/bin/bash

# Configuration
ISAACLAB_PATH="${AARON_WORKSPACE:-$HOME/aaron_workspace}/isaac_setup/IsaacLab/isaaclab.sh"
PROJECT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

# Colors for output
GREEN='\033[0;32m'
NC='\033[0m' # No Color

echo -e "${GREEN}== ARCPro RL Simulation Verification ==${NC}"
cd "$PROJECT_DIR"

if [ ! -f "$ISAACLAB_PATH" ]; then
    echo "Error: Isaac Lab not found at $ISAACLAB_PATH"
    exit 1
fi

echo "Running spawn and physics verification..."
"$ISAACLAB_PATH" -p arcproLab/scripts/verify_spawn.py --num_envs 1 --enable_cameras

echo -e "\n${GREEN}Verification script finished.${NC}"
