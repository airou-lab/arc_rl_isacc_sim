# Project: ARCPro RL Isaac Sim Migration

## Context
This project focuses on integrating a reinforcement learning (RL) policy with an ARCPro robot model in NVIDIA Isaac Sim using the **Isaac Lab** framework. Originally based on a ROS2-bridged system, it has been migrated to a vectorized `ManagerBasedRLEnv` for higher performance and stability.

## Current Setup
- **Simulation**: Isaac Sim / Isaac Lab environment.
- **Scene Assets**: USD files include `no_graph_sim_cleaned.usd` as the primary track.
- **Robot Configuration**: `arcpro_robot_cfg.py` defines the 34-joint ARCPro robot.
- **Environment**: `arcpro_env_cfg.py` implementing a Gymnasium interface via Isaac Lab Managers (Observation, Action, Reward, Termination).
- **Policy Framework**: Stable Baselines 3 (SB3).

## Success Criteria (v1.0 ACHIEVED)
1.  **Isaac Lab Migration**: Multi-robot parallel training enabled.
2.  **Physics Stability**: Stabilized robot joints and chassis scaling.
3.  **Vision Pipeline**: 160x90 RGB observation capture via `TiledCamera`.
4.  **Policy Control**: Autonomous lap completion using an SB3 policy.

## Tech Stack
- NVIDIA Isaac Sim / Isaac Lab
- Python 3.12
- Stable Baselines 3 (SB3)
- Gymnasium

