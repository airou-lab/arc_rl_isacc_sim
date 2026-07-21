#!/bin/bash
# Kill any existing training session
tmux kill-session -t training 2>/dev/null || true
pkill -f train_policy.py 2>/dev/null || true
pkill -f train_skrl.py 2>/dev/null || true

# Create a new tmux session named 'training' in the background
tmux new-session -d -s training

# Pane 0: Start the actual training, pipe output to tee so it logs AND shows on screen
tmux send-keys -t training:0.0 'export TORCH_USE_CUDA_DSA=1; export PYTHONUNBUFFERED=1; /home/arika/IsaacLab/isaaclab.sh -p arcproLab/scripts/train_skrl.py --num_envs 8 --headless --total_timesteps 5000000 | tee -a logs/skrl_phase1.log' C-m

# Pane 1: Split vertically and start the watchdog
tmux split-window -t training:0 -v
# Give the training a few seconds to create the log file before starting the watchdog
tmux send-keys -t training:0.1 'sleep 5 && python arcproLab/scripts/watchdog.py' C-m

echo "Started 8-env training + watchdog in tmux."
echo "Run 'tmux attach -t training' to view it live!"
