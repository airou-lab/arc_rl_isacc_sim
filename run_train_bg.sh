#!/bin/bash
# Start background training with 32 environments (Original Mastery Config)
nohup script -c "/home/arika/IsaacLab/isaaclab.sh -p arcproLab/scripts/train_policy.py --num_envs 32 --headless --total_timesteps 5000000 --enable_cameras" -f logs/production_mastery_32_relaunch.log > /dev/null 2>&1 &
echo "Training started in background (32 envs). Log: logs/production_mastery_32_relaunch.log"
