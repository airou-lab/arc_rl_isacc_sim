#!/bin/bash

# Configuration
MODEL="ollama/qwen2.5-coder:7b"
LOG_FILE="logs/latest_run.log"
SCRIPT_TO_RUN="arcproLab/scripts/train_skrl.py"
ENV_CFG="arcproLab/arcpro_env_cfg.py"
REWARDS="arcproLab/mdp/rewards.py"
CHECK_INTERVAL=7200  # 2 hours in seconds

echo "Starting Autonomous Self-Healing RL Pipeline..."
echo "Qwen check interval: $((CHECK_INTERVAL / 3600)) hours"

while true; do
    echo "=========================================="
    echo "Starting Isaac Lab Training..."
    echo "=========================================="
    
    # 1. Run training with a 4-hour timeout
    # Auto-find the absolute most recently modified checkpoint
    LATEST_CKPT=$(find logs/ppo_skrl -name "agent_*.pt" -printf "%T@ %p\n" | sort -n | tail -n 1 | awk '{print $2}')
    RESUME_FLAG=""
    if [ ! -z "$LATEST_CKPT" ]; then
        RESUME_FLAG="--resume $LATEST_CKPT"
        echo "Found checkpoint to resume: $LATEST_CKPT"
    fi

    # timeout returns exit code 124 when time expires
    # Stderr is redirected so Aider can read the crash logs in the file
    # We pipe to tee so it shows live on the screen, and use PIPESTATUS to get the true exit code
    timeout $CHECK_INTERVAL /home/arika/IsaacLab/isaaclab.sh -p $SCRIPT_TO_RUN --num_envs 32 --headless --enable_cameras $RESUME_FLAG 2>&1 | tee $LOG_FILE
    EXIT_CODE=${PIPESTATUS[0]}

    # 2. Free up VRAM by ensuring Isaac is fully killed
    pkill -f train_skrl.py
    sleep 5 

    # Determine reason for stop
    if [ $EXIT_CODE -eq 124 ]; then
        REASON="4-hour periodic check (timeout)"
    elif [ $EXIT_CODE -eq 0 ]; then
        echo "Training completed successfully (Exit Code 0). Exiting loop."
        break
    else
        REASON="crash (exit code $EXIT_CODE)"
    fi

    # 3. Trigger Local AI analysis
    echo "Training stopped: $REASON"
    echo "Waking up Qwen to analyze and fix..."
    
    # Create prompt for Qwen
    CRASH_LOG=$(tail -n 60 $LOG_FILE | tr -d '\0')
    PROMPT="The Isaac Lab training was stopped due to: $REASON. Here is the end of the log:\n\n$CRASH_LOG\n\nCRITICAL INSTRUCTIONS:\n1. You MUST be GSD (Get Shit Done) aware. Read all files in the '.planning/' and '.gsd/' directories first to understand the ongoing project context and past reward iterations.\n2. You are allowed and encouraged to use available debugging bash scripts in the workspace (like debug.sh, steer.sh, etc.) if you need to run tests.\n3. Make the minimal necessary changes to resolve the issue in the python configuration files.\n4. When you finish, you MUST update '.planning/reward_tuning_history.md' and '.planning/RESUME.md' with the changes you made.\n5. Leave a brief summary of your actions in a file called '.planning/QWEN_UPDATE.md' so that the main Antigravity agent can read it later when the user asks for a status update.\n6. If the agent is stagnating (WPs_cum near 0, Spd near 0), you MUST change the reward configuration to break out of the local minimum. Consult '.planning/reward_tuning_history.md' to avoid repeating past fixes."
    
    # 4. Use Aider CLI to automatically read log and edit files
    # --yes: autonomously accept code edits
    # --read: load all planning/GSD context so Qwen has full project awareness
    # --auto-test + --test-cmd: after every edit, Qwen runs debug.sh to verify the fix works.
    #   If it fails, Qwen reads the error output and iterates until it passes!
    ./venv/bin/aider \
      --model $MODEL \
      --yes \
      --auto-test \
      --test-cmd "timeout 120 bash debug.sh --num_envs 1 --headless --enable_cameras 2>&1 | tail -n 30" \
      --message "$PROMPT" \
      --read $(find .planning .gsd -type f -name "*.md") \
      $SCRIPT_TO_RUN $ENV_CFG $REWARDS .planning/RESUME.md .planning/reward_tuning_history.md .planning/QWEN_UPDATE.md
      
    echo "AI modifications completed."
    
    # 5. Unload the model to free VRAM for Isaac Lab
    curl -s -X POST http://localhost:11434/api/generate -d '{"model": "qwen2.5-coder:7b", "keep_alive": 0}'
    sleep 2
done
