
import os
import numpy as np
import gymnasium as gym
from gymnasium import spaces
import csv
import time
from datetime import datetime

# Isaac Sim imports must happen after SimulationApp is initialized
from isaacsim import SimulationApp

class IsaacDirectEnv(gym.Env):
    """
    A pure Isaac Sim Gymnasium environment for the F1Tenth robot.
    Bypasses ROS2 for direct tensor-based control and observation.
    Uses direct joint manipulation for maximum reliability.
    """
    metadata = {"render_modes": ["human"]}

    def __init__(self, usd_path=None, headless=False, control_hz=20, sub_steps=10, simulation_app=None):
        super(IsaacDirectEnv, self).__init__()

        # 1. Start / Attach Simulation App
        if simulation_app is not None:
            self._simulation_app = simulation_app
        else:
            self._simulation_app = SimulationApp({"headless": headless})
        
        # 2. Imports after app start
        import omni
        from omni.isaac.core import World
        from omni.isaac.core.articulations import Articulation
        from isaacsim.sensors.camera import Camera
        from omni.isaac.core.utils.rotations import quat_to_euler_angles
        from omni.isaac.core.utils.types import ArticulationAction
        
        self.omni = omni
        self.World = World
        self.Articulation = Articulation
        self.Camera = Camera
        self.quat_to_euler_angles = quat_to_euler_angles
        self.ArticulationAction = ArticulationAction

        # 3. Environment Config
        self.usd_path = usd_path or "/home/arika/Documents/arcpro/arcpro_system/src/examples/ARCPro_RL/arc_rl_isacc_sim/isacc_sim_usd/World0.usd"
        self.control_hz = control_hz
        self.sub_steps = sub_steps
        self.dt = 1.0 / self.control_hz
        self.max_speed = 3.0 
        self.max_steer = 0.5 
        
        # 4. Define Spaces
        self.action_space = spaces.Box(
            low=np.array([-1.0, 0.0, 0.0]), 
            high=np.array([1.0, 1.0, 1.0]), 
            dtype=np.float32
        )
        
        self.observation_space = spaces.Dict({
            "image": spaces.Box(low=0, high=255, shape=(90, 160, 3), dtype=np.uint8),
            "vec": spaces.Box(low=-np.inf, high=np.inf, shape=(12,), dtype=np.float32)
        })

        # 5. Load Stage
        if os.path.exists(self.usd_path):
            print(f"[IsaacDirectEnv] Loading stage: {self.usd_path}")
            self.omni.usd.get_context().open_stage(self.usd_path)
        else:
            raise FileNotFoundError(f"USD Stage not found at {self.usd_path}")

        # 6. Initialize World
        self.world = self.World(physics_dt=self.dt / self.sub_steps, rendering_dt=self.dt)

        # 7. Setup Robot
        self.robot_path = "/World/F1Tenth"
        self.robot = self.Articulation(self.robot_path)
        
        # Mapping for action application (Indices found during inspection)
        self.steering_indices = np.array([22, 19]) # Left, Right
        self.throttle_indices = np.array([25, 24, 16, 13]) # FL, FR, BL, BR

        # 8. Setup Camera
        self.camera_path = f"{self.robot_path}/Rigid_Bodies/Chassis/Camera_Left"
        self.camera = self.Camera(prim_path=self.camera_path, resolution=(160, 90))
        
        # 9. Internal State
        self.last_action = np.zeros(3, dtype=np.float32)
        self.total_distance = 0.0
        self.turn_bias = 0.0
        
        # 10. Logging Setup
        self.log_file = f"isaac_eval_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        self._init_logger()

        # Finalize
        self.world.scene.add(self.robot)
        self.world.reset()
        self.camera.initialize()
        
        print("[IsaacDirectEnv] Initialized successfully.")

    def _init_logger(self):
        with open(self.log_file, mode='w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(["timestamp", "step", "steer_cmd", "throttle_cmd", "brake_cmd", "actual_speed", "status"])

    def _log_step(self, step_count, action, actual_speed, status):
        with open(self.log_file, mode='a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([time.time(), step_count, action[0], action[1], action[2], actual_speed, status])

    def _get_observation(self):
        obs_image = self.camera.get_rgba()[:, :, :3]
        
        linear_velocity = self.robot.get_linear_velocity()
        if linear_velocity is None or np.isnan(linear_velocity).any():
            linear_velocity = np.zeros(3)
        actual_speed = np.linalg.norm(linear_velocity)
        
        angular_velocity = self.robot.get_angular_velocity()
        if angular_velocity is None or np.isnan(angular_velocity).any():
            angular_velocity = np.zeros(3)
        yaw_rate = angular_velocity[2]
        
        vec = np.array([
            self.turn_bias, 0.0, 0.0,
            float(actual_speed), float(yaw_rate),
            float(self.last_action[0]), float(self.last_action[1]), float(self.last_action[2]),
            0.0, 0.0, 0.0,
            float(self.total_distance)
        ], dtype=np.float32)
        
        return {"image": obs_image, "vec": vec}, actual_speed

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        self.world.reset()
        self.robot.initialize()
        
        # Stable spawn Pose
        pos = np.array([0, 0, 0.08])
        ori = np.array([1, 0, 0, 0])
        
        if options and "initial_state" in options:
            pos = options["initial_state"].get("position", pos)
            ori = options["initial_state"].get("orientation", ori)
        
        self.robot.set_world_pose(position=pos, orientation=ori)
        
        # Force a settle period WITH RENDERING to warm up camera
        self.robot.set_joint_velocities(np.zeros(self.robot.num_dof))
        for _ in range(50): self.world.step(render=True)
        
        self.last_action = np.zeros(3, dtype=np.float32)
        self.total_distance = 0.0
        self.step_count = 0
        self.world.step(render=True)
        obs, _ = self._get_observation()
        return obs, {}

    def step(self, action):
        self.step_count += 1
        self.last_action = action.copy()
        steer_cmd, throttle_cmd, brake_cmd = action

        # 1. Direct Joint Commands
        target_steer = steer_cmd * self.max_steer
        # Target velocity in Rad/s (100 rad/s is roughly 5 m/s for 0.05m radius wheels)
        target_vel = (throttle_cmd - brake_cmd) * 100.0
        
        # Add a minimum "kick" to overcome friction
        if throttle_cmd > 0.05 and np.abs(target_vel) < 15.0:
            target_vel = 15.0 if target_vel >= 0 else -15.0

        # Create Persistent Action
        # Steering (indices 22, 19) -> Position Control
        # Throttle (indices 25, 24, 16, 13) -> Velocity Control
        
        # We must provide arrays matching the size of the robot's TOTAL DOFs if we want to be safe, 
        # or use specific indices.
        
        # Method: Set only the relevant targets for the relevant indices
        art_action = self.ArticulationAction(
            joint_positions=np.array([target_steer, target_steer]),
            joint_indices=self.steering_indices
        )
        self.robot.apply_action(art_action)
        
        art_action_vel = self.ArticulationAction(
            joint_velocities=np.array([target_vel, target_vel, target_vel, target_vel]),
            joint_indices=self.throttle_indices
        )
        self.robot.apply_action(art_action_vel)

        # 2. Physics Step
        for _ in range(self.sub_steps):
            self.world.step(render=True)

        # 3. Observations & Failures
        obs, actual_speed = self._get_observation()
        self.total_distance += actual_speed * self.dt
        
        pos, quat = self.robot.get_world_pose()
        roll, pitch, yaw = self.quat_to_euler_angles(quat)
        roll_deg, pitch_deg = np.rad2deg(roll), np.rad2deg(pitch)
        
        terminated = False
        status = "Active"
        if pos[2] < -0.5:
            terminated = True
            status = "Fallen"
        elif np.abs(roll_deg) > 45 or np.abs(pitch_deg) > 45:
            terminated = True
            status = "Flipped"
        
        if self.step_count % 10 == 0:
            print(f"Step: {self.step_count} | Speed: {actual_speed:.2f} m/s | Status: {status}")
        
        return obs, 0.0, terminated, False, {"speed": actual_speed, "status": status}

    def close(self):
        self._simulation_app.close()
