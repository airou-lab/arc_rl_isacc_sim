# Tech Stack

## Core Technologies
- **Simulation Platform:** NVIDIA Isaac Sim (2022.2.1+)
- **RL Framework:** Isaac Lab (Manager-Based API)
- **Deep Learning:** PyTorch (for rewards and observations)
- **Middleware:** ROS 2 Jazzy/Humble (via `rclpy`)
- **Physics Engine:** NVIDIA PhysX
- **Environment Management:** Gymnasium (SB3 compatible)

## Supporting Tools
- **Testing:** pytest (with `unittest.mock` for Isaac Sim dependencies)
- **Linting:** flake8
- **Visualization:** Matplotlib (for track verification plots)
- **Data:** NumPy 1.x (for waypoint management)
