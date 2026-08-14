# Scripts

Executable entry points for training, testing, and debugging the environment.

- `train_skrl.py`: The main entry point to launch PPO training.
- `play_skrl.py`: Run a fully trained policy for evaluation.
- `teleop.py`: Control the robot manually for debugging.
- `watchdog.py`: A background daemon to monitor and auto-fix RL stagnation.
- `audit_*.py`, `verify_*.py`, `test_*.py`: Isolated scripts for debugging specific mechanics (vision, bounds, joints, etc).
