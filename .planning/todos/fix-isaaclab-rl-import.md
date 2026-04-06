---
area: training
milestone: 1
phase: 07
status: open
priority: high
created_at: 2026-04-05
---

# Todo: Fix Isaac Lab RL Import Error

## Issue
Running `train_policy.py` fails with:
`ModuleNotFoundError: No module named 'isaaclab_rl.stable_baselines3'`

## Context
- File: `arcproLab/scripts/train_policy.py`
- Line: 44 (approximately)
- Likely cause: The SB3 wrapper path has changed or the `isaaclab_rl` extension is not enabled/installed in the current Isaac Lab environment.

## Action
- Verify the correct import path for `Sb3VecEnvWrapper` in the current Isaac Lab installation.
- Update `train_policy.py` with the correct path.
- Verify that `isaaclab_rl` extension is active.
