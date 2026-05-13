#!/bin/bash
# Start background training with 16 environments (Optimized for RTX 3060 VRAM)
nohup script -c "/home/arika/IsaacLab/isaaclab.sh -p arcproLab/scripts/train_policy.py --num_envs 16 --headless --total_timesteps 5000000 --enable_cameras" -f logs/production_training_16.log > /dev/null 2>&1 &
echo "Training started in background (16 envs). Log: logs/production_training_16.log"
