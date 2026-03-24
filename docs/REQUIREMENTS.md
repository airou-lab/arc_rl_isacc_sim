# Requirements: RL Policy Integration

## Functional Requirements
- **FR1**: Isaac Sim must load the specified USD stage (`World0.usd` or `jetracer_track.usd`) with the robot.
- **FR2**: The robot in simulation must be controllable via `/ackermann_cmd` ROS2 topic.
- **FR3**: The simulation must publish camera images to `/camera/image_raw` and vehicle state to `/vehicle_state`.
- **FR4**: The RL policy must be loaded from a `.zip` or `.pth` file using Stable Baselines 3.
- **FR5**: The system must run a closed-loop control where the policy takes observations from the `IsaacROS2Env` and sends actions back to simulation.

## Technical Requirements
- **TR1**: ROS2 bridge must be active in Isaac Sim (`isaacsim.ros2.bridge`).
- **TR2**: Python environment must have `gymnasium`, `stable_baselines3`, `rclpy`, and `cv_bridge` installed.
- **TR3**: Inference script must handle the `Dict` observation space defined in `isaac_ros2_env.py`.
- **TR4**: Action Graph in Isaac Sim must map `/ackermann_cmd` to the robot's articulation (steering and throttle).
- **TR5**: Action Graph in Isaac Sim must publish the camera feed from the robot's onboard camera to `/camera/image_raw`.
