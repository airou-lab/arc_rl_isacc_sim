#!/bin/bash
# If invoked from inside tmux session 'training', detach cleanly without inheriting $TMUX
if [ -n "$TMUX" ] && [ "$1" != "--detached-internal" ]; then
    if tmux display-message -p '#S' 2>/dev/null | grep -q "^training$"; then
        echo "[start_tmux] Detaching from active session 'training' to execute restart..."
        env -u TMUX -u TMUX_PANE nohup bash "$0" --detached-internal "$@" > /tmp/start_tmux.log 2>&1 &
        exit 0
    fi
fi
if [ "$1" = "--detached-internal" ]; then
    shift
fi

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

# Automatically resume from specified checkpoint or the newest saved checkpoint
if [ -n "$1" ]; then
    LATEST_CKPT="$1"
else
    BRANCH=$(git rev-parse --abbrev-ref HEAD 2>/dev/null || echo "main")
    BRANCH_TAG=$(echo "$BRANCH" | tr '/' '_')
    LOG_SUBDIR="logs/ppo_skrl_${BRANCH_TAG}"
    if [ "$BRANCH" = "main" ] || [ ! -d "$LOG_SUBDIR" ]; then
        LOG_SUBDIR="logs/ppo_skrl"
    fi
    LATEST_CKPT=$(find "$LOG_SUBDIR" -name "agent_*.pt" -printf "%T@ %p\n" 2>/dev/null | sort -n | tail -n 1 | awk '{print $2}')
    if [ -z "$LATEST_CKPT" ]; then
        LATEST_CKPT=$(find "$LOG_SUBDIR" -name "best_agent.pt" -printf "%T@ %p\n" 2>/dev/null | sort -n | tail -n 1 | awk '{print $2}')
    fi
fi
RESUME_FLAG=""
if [ ! -z "$LATEST_CKPT" ]; then
    RESUME_FLAG="--resume $LATEST_CKPT"
    echo "Resuming training from checkpoint: $LATEST_CKPT"
fi

# Pane 0: Start the actual training, pipe output to tee so it logs AND shows on screen
tmux send-keys -t training:0.0 "export TORCH_USE_CUDA_DSA=1; export PYTHONUNBUFFERED=1; /home/arika/IsaacLab/isaaclab.sh -p arcproLab/scripts/train_skrl.py --num_envs 20 --headless --enable_cameras --total_timesteps 5000000 $RESUME_FLAG | tee logs/skrl_phase1.log" C-m

# Pane 1: Split vertically and start the autonomous self-healing sentinel
tmux split-window -t training:0 -v
# Give Isaac Sim 20 seconds to initialize simulation before starting the sentinel
tmux send-keys -t training:0.1 'sleep 20 && python3 arcproLab/scripts/autonomous_sentinel.py --daemon --interval 15.0' C-m

echo "Started 20-env SKRL training (~10GB VRAM) + autonomous sentinel in tmux."
echo "Run 'tmux attach -t training' to view it live!"
