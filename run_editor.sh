#!/bin/bash
# Path to Isaac Sim binary
ISAAC_SIM_EXE="/home/arika/isaac-sim-5.1.0/isaac-sim.sh"
USD_PATH="$(pwd)/openStreetUSD/no_graph_sim_hardened.usd"

echo "--------------------------------------------------"
echo "Opening Isaac Sim Editor"
echo "Target: $USD_PATH"
echo "--------------------------------------------------"

$ISAAC_SIM_EXE "$USD_PATH"
