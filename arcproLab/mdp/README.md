# MDP (Markov Decision Process)

This folder contains the core logic for the reinforcement learning environment's MDP components. Following Grug Brain principles, each file is kept flat, readable, and focused on one aspect of the MDP without unnecessary abstraction layers.

- `actions.py`: Translates raw NN outputs into physical joint commands (Grouped joints, Ackermann steering).
- `observations.py`: Translates physical state and camera renders into NN inputs.
- `rewards.py`: Defines the dense reward shaping functions to guide PPO.
- `terminations.py`: Defines episode-ending conditions (e.g., crashing, finishing).
- `track_manager.py`: Handles high-fidelity track boundary extraction and waypoint sequence generation.
- `go_signal_manager.py`, `road_manager.py`, `events.py`: Focused helper logic for environment interactions.
