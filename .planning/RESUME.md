# IsaacLab F1Tenth RL Training - Resume Context

## Current Status
We successfully debugged a major kinematics failure where the F1Tenth vehicle was acting like a forklift (driving backwards with locked front wheels). 
The physics are now visually confirmed to be stable. The car is properly driving in All-Wheel Drive (AWD) and steering correctly using Ackermann kinematics.
A fresh SKRL training run (16-env) has been started via tmux because the previous neural network checkpoints were incompatible with the corrected physics.

## Most Recent Changes (Last Session)
1. **Fixed "Forklift" Reverse Driving:** In `arcpro_env_cfg.py`, the `b_drive` action space scale/offset was reverted from `20.0` to `-20.0`. The car now drives forward relative to its mesh, placing the steering axle correctly at the front (restoring Front-Wheel Steering).
2. **Fixed "Parking Brake" Front Wheels:** Reverted the RWD experiment back to **AWD**. The `b_drive` and `throttle` actuators in `arcpro_env_cfg.py` and `arcpro_robot_cfg.py` were updated to target all four wheels (`["Joint_Drive_.*"]`). This prevents the front wheels from acting as unactuated parking brakes dragging on the pavement.
3. **Updated Test Scripts:** Created `steer.sh` and `play_steer.py` (and updated `play_straight.py`) to correctly interface with the 2D action space `[steer, drive]`. The user successfully visually verified the car driving straight and steering.
4. **Wiped Checkpoints:** Deleted the obsolete neural network checkpoints in `logs/ppo_skrl/` that were trained on the broken backwards physics.
5. **Started Fresh Training:** Executed `./start_tmux_training.sh`. The training and watchdog are currently running in a background tmux session named `training`.

## Active Issues / Blockers
- **None at the moment.** The training is running. We must wait to see if the agent can overcome the first curve with the corrected physics. 

## Next Steps for New Agent
1. Read `.planning/monitor_agent_prompt.md` to understand the watchdog rules.
2. Ask the user if they want to check the status of the training by attaching to the tmux session or if they want to spawn a monitor subagent to periodically check the tensorboard logs.
3. Do NOT modify the physics or drive actions. They have been visually verified to work!
4. If the agent is crashing early, focus strictly on **reward tuning** (using `reward_tuning_history.md` as a guide) rather than changing the robot's physical attributes.
