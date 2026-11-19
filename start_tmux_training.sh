#!/bin/bash
# If invoked from inside tmux session 'training', detach execution to prevent suicide
if [ -n "$TMUX" ] && [ "${RUNNING_DETACHED:-0}" != "1" ]; then
    CURRENT_SESSION=$(tmux display-message -p '#S' 2>/dev/null || true)
    if [ "$CURRENT_SESSION" = "training" ]; then
        RUNNING_DETACHED=1 env -u TMUX nohup bash "$0" "$@" > /tmp/start_tmux.log 2>&1 &
        exit 0
    fi
fi

# Parse CLI flags: --fresh / --scratch / --clean, --resume <ckpt>, --vision_mode {rgb,yuv}, or positional <ckpt>
FRESH_START=0
EXPLICIT_CKPT=""
VISION_MODE="rgb"

while [ $# -gt 0 ]; do
    case "$1" in
        --fresh|--scratch|clean|--clean)
            FRESH_START=1
            shift
            ;;
        --resume)
            if [ -n "${2:-}" ]; then
                EXPLICIT_CKPT="$2"
                shift 2
            else
                echo "Error: --resume requires a checkpoint path." >&2
                exit 1
            fi
            ;;
        --vision_mode|--vision-mode)
            if [ -n "${2:-}" ]; then
                VISION_MODE="$2"
                shift 2
            else
                echo "Error: --vision_mode requires 'rgb' or 'yuv'." >&2
                exit 1
            fi
            ;;
        *)
            EXPLICIT_CKPT="$1"
            shift
            ;;
    esac
done

# Kill any existing training session using bracket-escaped regexes
tmux kill-session -t training 2>/dev/null || true
pkill -f "[t]rain_policy.py" 2>/dev/null || true
pkill -f "[t]rain_skrl.py" 2>/dev/null || true

# Resolve active branch and log paths without falling back to stale legacy logs
mkdir -p logs
LOG_FILE="logs/skrl_phase1.log"
if [ -f "$LOG_FILE" ]; then
    mv "$LOG_FILE" "logs/skrl_phase1_$(date +%Y%m%d_%H%M%S).log"
fi

BRANCH=$(git rev-parse --abbrev-ref HEAD 2>/dev/null || echo "main")
BRANCH_TAG=$(echo "$BRANCH" | tr '/' '_')
LOG_SUBDIR="logs/ppo_skrl_${BRANCH_TAG}"

# Create a new tmux session named 'training' in the background
tmux new-session -d -s training

# Resolve checkpoint or fresh start
if [ "$FRESH_START" = "1" ]; then
    LATEST_CKPT=""
    ./venv/bin/python - "$LOG_SUBDIR" <<'PYEOF'
import json, os, sys, time
from pathlib import Path
log_subdir = sys.argv[1]
audited = []
if os.path.isdir(log_subdir):
    for root, _, files in os.walk(log_subdir):
        for f in files:
            if f.startswith("agent_") and f.endswith(".pt"):
                p = Path(root) / f
                run_dir = p.parent.parent if p.parent.name == "checkpoints" else p.parent
                audited.append(f"{run_dir.name}/{p.name}")
os.makedirs("logs", exist_ok=True)
with open("logs/sentinel_state.json", "w") as fp:
    json.dump({"audited_checkpoints": sorted(set(audited)), "session_start_time": time.time()}, fp, indent=2)
PYEOF
    echo "Starting clean training from step 0 (fresh start, vision_mode=$VISION_MODE)."
elif [ -n "$EXPLICIT_CKPT" ]; then
    LATEST_CKPT="$EXPLICIT_CKPT"
else
    if [ -d "$LOG_SUBDIR" ]; then
        LATEST_CKPT=$(find "$LOG_SUBDIR" -name "agent_*.pt" ! -path "*corrupted*" -printf "%T@ %p\n" 2>/dev/null | sort -n | tail -n 1 | awk '{print $2}')
        if [ -z "$LATEST_CKPT" ]; then
            LATEST_CKPT=$(find "$LOG_SUBDIR" -name "best_agent.pt" ! -path "*corrupted*" -printf "%T@ %p\n" 2>/dev/null | sort -n | tail -n 1 | awk '{print $2}')
        fi
    else
        LATEST_CKPT=""
    fi
    if [ -z "$LATEST_CKPT" ]; then
        echo "No active checkpoints found in $LOG_SUBDIR. Starting fresh training (vision_mode=$VISION_MODE)."
    fi
fi

TRAIN_FLAGS="--vision_mode $VISION_MODE"
if [ "$FRESH_START" = "1" ]; then
    TRAIN_FLAGS="$TRAIN_FLAGS --fresh"
elif [ -n "$LATEST_CKPT" ]; then
    TRAIN_FLAGS="$TRAIN_FLAGS --resume $LATEST_CKPT"
    echo "Resuming training from checkpoint: $LATEST_CKPT (vision_mode=$VISION_MODE)"
fi

# Pane 0: Start the actual training, pipe output to tee so it logs AND shows on screen
tmux send-keys -t training:0.0 "export PYTORCH_CUDA_ALLOC_CONF=expandable_segments:True; export PYTHONUNBUFFERED=1; /home/arika/IsaacLab/isaaclab.sh -p arcproLab/scripts/train_skrl.py --num_envs 48 --headless --enable_cameras --total_timesteps 5000000 $TRAIN_FLAGS 2>&1 | tee $LOG_FILE; echo 'Training process finished with exit code '\$?; bash" C-m

# Pane 1: Split vertically and start the autonomous self-healing sentinel with explicit log paths
tmux split-window -t training:0 -v
tmux send-keys -t training:0.1 "sleep 20 && ./venv/bin/python arcproLab/scripts/autonomous_sentinel.py --daemon --interval 30.0 --log-dir $LOG_SUBDIR --log-file $LOG_FILE" C-m

echo "Started 48-env SKRL training (vision_mode=$VISION_MODE) + autonomous sentinel in tmux."
echo "Run 'tmux attach -t training' to view it live!"
