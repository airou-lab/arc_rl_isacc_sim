#!/bin/bash
# Kill any existing training session
tmux kill-session -t training 2>/dev/null || true
pkill -f train_policy.py 2>/dev/null || true
pkill -f train_skrl.py 2>/dev/null || true

# Backup previous training log if it exists
mkdir -p logs
if [ -f "logs/skrl_phase1.log" ]; then
    mv logs/skrl_phase1.log "logs/skrl_phase1_$(date +%Y%m%d_%H%M%S).log"
fi

# Create a new tmux session named 'training' in the background
tmux new-session -d -s training

# Auto-find the absolute most recently modified checkpoint
LATEST_CKPT=$(find logs/ppo_skrl -name "agent_*.pt" -printf "%T@ %p\n" | sort -n | tail -n 1 | awk '{print $2}')
RESUME_FLAG=""
if [ ! -z "$LATEST_CKPT" ]; then
    RESUME_FLAG="--resume $LATEST_CKPT"
    echo "Found checkpoint to resume: $LATEST_CKPT"
fi

# Pane 0: Start the actual training, pipe output to tee so it logs AND shows on screen
tmux send-keys -t training:0.0 "export TORCH_USE_CUDA_DSA=1; export PYTHONUNBUFFERED=1; /home/arika/IsaacLab/isaaclab.sh -p arcproLab/scripts/train_skrl.py --num_envs 32 --headless --enable_cameras --total_timesteps 5000000 $RESUME_FLAG | tee logs/skrl_phase1.log" C-m

# Pane 1: Split vertically and start the watchdog
tmux split-window -t training:0 -v
# Give Isaac Sim 20 seconds to initialize simulation before starting the watchdog
tmux send-keys -t training:0.1 'sleep 20 && tail -f logs/skrl_phase1.log | python arcproLab/scripts/watchdog.py' C-m

echo "Started 10-env SKRL training + watchdog in tmux."
echo "Run 'tmux attach -t training' to view it live!"
