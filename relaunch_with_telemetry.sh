#!/bin/bash
# Relaunch training from the 3.5M step checkpoint with new telemetry logging
nohup script -c "/home/arika/IsaacLab/isaaclab.sh -p arcproLab/scripts/train_policy.py --num_envs 32 --headless --total_timesteps 5000000 --enable_cameras --checkpoint logs/ppo/20260516-064612/model_3500000_steps.zip" -f logs/production_mastery_32_telemetry.log > /dev/null 2>&1 &
echo "Training resumed in background with telemetry. Log: logs/production_mastery_32_telemetry.log"
