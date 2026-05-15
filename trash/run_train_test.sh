#!/bin/bash
# 1 instance training in gui foreground for testing
/home/arika/IsaacLab/isaaclab.sh -p arcproLab/scripts/train_policy.py --num_envs 1 --total_timesteps 5000000 --enable_cameras
