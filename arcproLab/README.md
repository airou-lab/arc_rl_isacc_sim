# ARCPro RL Environment

This directory contains the core IsaacLab environment and training logic for the ARCPro Self-Driving Agent.
We adhere to the "Grug Brained Developer" philosophy: flat structures, single responsibility files, and minimal abstraction (no unnecessary enterprise design patterns).

## Subdirectories
- `mdp/`: Contains all Markov Decision Process logic. Everything is broken down cleanly into specific files: `actions.py`, `observations.py`, `rewards.py`, `terminations.py`, and focused utilities.
- `agents/`: Contains the PPO implementation and neural network architecture wrappers.
- `scripts/`: Contains executable entry points for training, inference, and debugging.
- `assets/` & `models/`: Additional resources and 3D assets required for simulation.
