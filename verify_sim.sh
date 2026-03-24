#!/bin/bash

# Configuration
ISAACLAB_PATH="/home/arika/IsaacLab/isaaclab.sh"
PROJECT_DIR="/home/arika/Documents/arcpro/arcpro_system/src/examples/ARCPro_RL/arc_rl_isacc_sim"

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
"$ISAACLAB_PATH" -p arcproLab/scripts/verify_spawn.py --num_envs 1

echo -e "\n${GREEN}Verification script finished.${NC}"
