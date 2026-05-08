#!/bin/bash
# Start background training with 32 environments
nohup script -c "/home/arika/IsaacLab/isaaclab.sh -p arcproLab/scripts/train_policy.py --num_envs 30 --headless --total_timesteps 5000000 --enable_cameras" -f logs/production_training_32.log > /dev/null 2>&1 &
echo "Training started in background (32 envs). Log: logs/production_training_32.log"
