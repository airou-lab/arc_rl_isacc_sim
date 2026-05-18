#!/bin/bash
# Relaunch from 3.5M checkpoint. 
# Target: 5M total. 
# Delta: 1,500,000 steps.
nohup script -c "/home/arika/IsaacLab/isaaclab.sh -p arcproLab/scripts/train_policy.py --num_envs 32 --headless --total_timesteps 1500000 --enable_cameras --checkpoint logs/ppo/20260516-064612/model_3500000_steps.zip" -f logs/production_mastery_5M_target.log > /dev/null 2>&1 &
echo "Training resumed. Target: 5,000,000 Total Steps. Log: logs/production_mastery_5M_target.log"
