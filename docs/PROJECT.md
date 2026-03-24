# Project: Link RL Policy to Isaac Sim Robot

## Context
The goal is to integrate an existing Reinforcement Learning (RL) policy (from the `ARCPro_RL` project) with a robot model in NVIDIA Isaac Sim. The integration uses ROS2 as a bridge between the simulator and the RL environment.

## Current Setup
- **Simulation**: Isaac Sim installation in `/home/arika/nvida_isacc`. USD files include `jetracer_track.usd` and `World0.usd`.
- **RL Project**: `ARCPro_RL` (located in `Documents/arcpro/arcpro_system/src/examples/ARCPro_RL`).
- **Environment**: `isaac_ros2_env.py` implementing a Gymnasium interface via ROS2 topics (`/camera/image_raw`, `/vehicle_state`, `/ackermann_cmd`).
- **Policy Framework**: Stable Baselines 3 (indicated by `venv_isaac` packages).

## Goals
1.  Verify the ROS2 bridge configuration in Isaac Sim for the target robot.
2.  Locate or create an inference script to load the RL policy.
3.  Link the policy to the `IsaacROS2Env` and demonstrate robot control in simulation.

## Tech Stack
- NVIDIA Isaac Sim
- ROS2 (Humble/Foxy)
- Python 3.12
- Gymnasium
- Stable Baselines 3
