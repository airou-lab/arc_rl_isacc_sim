# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import os
import sys
import torch

sys.path.append(os.path.dirname(os.path.abspath(__file__)))

# Define base directories
ARCPRO_LAB_DIR = os.path.dirname(os.path.abspath(__file__))
USD_DIR = os.path.join(ARCPRO_LAB_DIR, "..", "openStreetUSD")

from isaaclab.utils import configclass
from isaaclab.envs import ManagerBasedRLEnvCfg, ViewerCfg
from isaaclab.managers import ObservationGroupCfg, ObservationTermCfg as ObsTerm, ActionTermCfg as ActionTerm, RewardTermCfg as RewTerm, TerminationTermCfg as DoneTerm, SceneEntityCfg, EventTermCfg
from isaaclab.assets import AssetBaseCfg
from isaaclab.sensors import TiledCameraCfg, ContactSensorCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sim import SimulationCfg
import isaaclab.sim as sim_utils
import isaaclab.envs.mdp as mdp

from arcpro_robot_cfg import ARCPRO_ROBOT_CFG
import mdp.observations as mdp_obs, mdp.rewards as mdp_rew, mdp.terminations as mdp_done, mdp.events as mdp_events
import mdp.spawner as arcpro_spawner
from mdp.debug_terminations import debug_termination

@configclass
class EventCfg:
    """Configuration for events."""
    # Snapped spawn: Uses raycasting to find the road height
    reset_robot_to_fixed_spawn = EventTermCfg(
        func=mdp_events.reset_robot_to_fixed_spawn,
        mode="reset",
        params={"asset_cfg": SceneEntityCfg("robot")},
    )

@configclass
class ARCProSceneCfg(InteractiveSceneCfg):
    """GSD Phase 7: True Physics Mode (1.0x Scale)."""
    
    # Lighting (Distant + Dome for full ambient visibility)
    light = AssetBaseCfg(
        prim_path="/World/light",
        spawn=sim_utils.DistantLightCfg(intensity=1500.0, color=(1.0, 1.0, 1.0)),
    )

    # Ground Plane (Visual + Safety Backup)
    ground_plane = AssetBaseCfg(
        prim_path="/World/defaultGroundPlane",
        spawn=sim_utils.GroundPlaneCfg(size=(200.0, 200.0)),
        init_state=AssetBaseCfg.InitialStateCfg(pos=(0.0, 0.0, -0.50)),
    )

    # Track from no_graph_sim.usd (Clean 1x version: No grass/foliage/fences)
    track = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Track",
        spawn=sim_utils.UsdFileCfg(
            usd_path=os.path.join(USD_DIR, "track_1x_wrapper.usda"),
            scale=(1.0, 1.0, 1.0),
        ),
        # Use origin position to match USD world coordinates
        init_state=AssetBaseCfg.InitialStateCfg(pos=(0.0, 0.0, 0.0)),
    )


    
    # Robot (1.0x scale on 1.0x track = Large world proportions)
    robot = ARCPRO_ROBOT_CFG.replace(
        prim_path="{ENV_REGEX_NS}/Robot",
        spawn=ARCPRO_ROBOT_CFG.spawn.replace(
            usd_path=os.path.join(ARCPRO_LAB_DIR, "assets", "robot", "F1Tenth_Metric.usd"),
            scale=(1.0, 1.0, 1.0), # Revert to 1.0x
        ),
        init_state=ARCPRO_ROBOT_CFG.init_state.replace(
            # Spawn point: right lane heading toward junction_18 intersection.
            # Exactly on the path center (X=-16.197) at safe drop height
            pos=(-16.197, 5.50, 0.5),
            rot=(0.7071, 0.0, 0.0, -0.7071), # Upright & Face South (WXYZ)
            # No kickstart — proper tire friction + drive torque is sufficient.
            # Kickstart was masking physics issues (bouncing, height terminations).
        ),



    )

    # Camera (Mimic Intel RealSense D435i Wide - 90° HFOV) with NO TILT
    tiled_camera = TiledCameraCfg(
        prim_path="{ENV_REGEX_NS}/Robot/Chassis/CameraSensor",
        update_period=0.0,
        spawn=sim_utils.PinholeCameraCfg(
            horizontal_aperture=3.86, # 90-degree HFOV (2 * atan(3.86 / (2 * 1.93)))
            focal_length=1.93,
        ),
        # Raised to 0.35m with no tilt to see further down the track (convention="ros" aligns camera forward)
        # Moved forward to pos=(0.5, 0.0, 0.35) to ensure it is completely outside the car mesh
        # The car drives in +X direction. We place it at +0.5 to clear the chassis mesh.
        # We pitch down 15 degrees around Y (w=0.9914, y=0.1305).
        offset=TiledCameraCfg.OffsetCfg(pos=(0.5, 0.0, 0.35), rot=(0.9914, 0.0, 0.1305, 0.0), convention="world"),
        data_types=["rgb"], width=224, height=224,
    )

    # Visual Marker for Camera (Tiny red cone, virtually zero mass, no collision)
    camera_marker = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Robot/Chassis/CameraMarker",
        spawn=sim_utils.ConeCfg(
            radius=0.03, # 3cm base
            height=0.1,  # 10cm long
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(1.0, 0.0, 0.0), emissive_color=(1.0, 0.0, 0.0)),
            rigid_props=None,
            collision_props=sim_utils.CollisionPropertiesCfg(collision_enabled=False),
            mass_props=sim_utils.MassPropertiesCfg(mass=0.001) # Explicitly 1 gram so it doesn't tip the car
        ),
        init_state=AssetBaseCfg.InitialStateCfg(
            pos=(0.3, 0.0, 0.35),
            rot=(0.707, 0.0, 0.707, 0.0) # Points X forward
        )
    )

    # Contact Sensor: Detect chassis collisions (crashes)
    contact_forces = ContactSensorCfg(
        prim_path="{ENV_REGEX_NS}/Robot/Chassis",
        update_period=0.0,
        history_length=3,
        debug_vis=False,
    )

    # Ground Contact Sensor: Detects any physical collision between the entire robot (.*) and the groundplane USD
    ground_contact = ContactSensorCfg(
        prim_path="{ENV_REGEX_NS}/Robot/.*",
        filter_prim_paths_expr=["/World/defaultGroundPlane"],
        update_period=0.0,
        history_length=3,
        debug_vis=False,
    )

@configclass
class ObservationCfg:
    @configclass
    class PolicyCfg(ObservationGroupCfg):
        concatenate_terms = False
        telemetry = ObsTerm(func=mdp_obs.get_telemetry_vector)
        tiled_camera = ObsTerm(func=mdp.image, params={"sensor_cfg": SceneEntityCfg("tiled_camera"), "normalize": True})
    policy: PolicyCfg = PolicyCfg()

    @configclass
    class CriticCfg(ObservationGroupCfg):
        telemetry = ObsTerm(func=mdp_obs.get_critic_state_vector)
    critic: CriticCfg = CriticCfg()

import mdp.actions as arcpro_actions

@configclass
class ActionCfg:
    a_steering = arcpro_actions.AckermannSteeringActionCfg(
        asset_name="robot",
        joint_names=["Joint_Steer_L", "Joint_Steer_R"], 
        wheelbase=0.33, 
        track_width=0.28, 
        max_steering_angle=0.5,
        offset=-0.005
    )
    b_drive = arcpro_actions.GroupedJointVelocityActionCfg(
        asset_name="robot", 
        joint_names=["Joint_Drive_.*"], 
        scale=20.0, 
        offset=20.0
    )

@configclass
class RewardCfg:
    # Survival Bonus: Disabled completely (weight 0.0). We rely entirely on bounded progress reward.
    # A positive survival bonus caused the agent to spin in circles to farm "living time" without making progress.
    survival_bonus = RewTerm(func=lambda env: torch.ones(env.num_envs, device=env.device), weight=0.0)

    # Anti-Suicide: Restored to 10.0 (-500) because weak penalties (-100) caused the agent to lazily crash without learning to steer.
    termination_penalty = RewTerm(func=mdp_rew.termination_penalty, weight=10.0)
    
    # Bound the waypoint progress reward with tanh.
    # At 0.5 m/s (~1.7 WPs), tanh(1.7) = 0.93.
    # At 1.5 m/s (~5.1 WPs), tanh(5.1) = 0.99.
    # This bootstraps the learning of cornering while the tanh bound
    # prevents the endless runner exploit because sprinting faster yields minimal extra points.
    progress_reward = RewTerm(func=lambda env: torch.tanh(mdp_rew.waypoint_progress_reward(env)), weight=200.0)
    
    # Exploration: Tiny reward for applying any throttle/drive action to break out of stagnation
    # Increased steering penalty (-2.0) to prevent the agent from exploiting spinning in circles (Spinning Top exploit).
    # Spinning for 400 steps will now cost -800 points, which is worse than crashing (-500), forcing forward progress!
    action_steer_penalty = RewTerm(func=lambda env: torch.square(env.action_manager.action[:, 0]), weight=0.0) # Disabled for Phase 1

    # Re-enabled (weight=0.5) to give the agent a hint to press the gas. 
    # The new 6-second stagnation termination prevents this from being farmed infinitely.
    action_drive_reward = RewTerm(
        func=lambda env: env.action_manager.action[:, 1],
        weight=0.5
    )
    
    # Precision: Lane centering (Disabled for Phase 1 Curriculum)
    lateral_error = RewTerm(func=mdp_rew.lateral_error_reward, weight=0.0)
    
    # Force movement: Penalty forces it to move!
    # Uses absolute speed so reverse driving doesn't spuriously trigger penalty.
    # Requires min speed of 0.5 m/s, UNLESS the traffic light (go_signal) says to stop.
    # Heavily penalized to prevent the Stationary Min-Max Trap.
    stationary = RewTerm(func=mdp_rew.stationary_penalty, weight=100.0)
    
    # Phase 2: Waypoint Tracking
    # Increased heading weight to 10.0. This acts as a 'Conditional Survival Bonus'.
    # It densely rewards the agent for facing forward (+10/step) and heavily penalizes facing backward (-10/step).
    # This prevents spin-outs and backwards driving without explicitly penalizing the steering action itself.
    heading = RewTerm(func=mdp_rew.heading_alignment_reward, weight=10.0)
    smoothness = RewTerm(func=mdp_rew.action_rate_smoothness_reward, weight=2.0)
    
    # Jitter Suppression (Disabled for Phase 1)
    jerk = RewTerm(func=mdp_rew.jerk_penalty, weight=0.0)
    
    # Boundary Penalty: (Enabled: Risk-aware shaping to smoothly steer away from walls)
    # Set to 0.4 (Issue 33) to perfectly balance against stagnation without over-saturation.
    boundary = RewTerm(func=mdp_rew.boundary_penalty, weight=0.4)


@configclass
class TerminationCfg:
    roadmark_contact = DoneTerm(func=mdp_done.white_line_contact, params={"threshold": 0.15})
    # Physical USD Contact: Terminate if any part of the robot touches the physical ground plane.
    # NOTE FOR USER: Threshold MUST be > 50.0! The track mesh is thin and rests 5cm above the ground plane.
    # The car's wheels slightly clip through the track mesh and constantly touch the ground plane beneath it,
    # generating small contact forces (e.g., 5-20N). A threshold of 1.0 triggers instantly on spawn.
    # A threshold of 50.0+ ignores the clipping wheels but will successfully trigger when the full weight
    # of the car slams into the ground plane after falling off the track!
    ground_contact_term = DoneTerm(func=mdp_done.ground_contact_termination, params={"sensor_cfg": SceneEntityCfg("ground_contact"), "threshold": 150.0})
    # Stagnation: Reset if stuck against a wall
    stagnation = DoneTerm(func=mdp_done.stagnation_termination)

@configclass
class ARCProEnvCfg(ManagerBasedRLEnvCfg):
    # Adjust viewer to see the robot at its new world position
    # 8x eye: (-120.0, 55.0, 10.0) -> 1x eye: (-15.0, 6.875, 1.25)
    viewer: ViewerCfg = ViewerCfg(eye=(-15.0, 6.25, 1.25), lookat=(-16.25, 4.92, 0.0))
    
    enable_cameras: bool = True
    scene: ARCProSceneCfg = ARCProSceneCfg(num_envs=32, env_spacing=30.0)
    observations: ObservationCfg = ObservationCfg()
    actions: ActionCfg = ActionCfg()
    rewards: RewardCfg = RewardCfg()
    terminations: TerminationCfg = TerminationCfg()
    events: EventCfg = EventCfg()
    sim: SimulationCfg = SimulationCfg(
        dt=0.002, # 500Hz for high-fidelity small scale physics
        render_interval=10, # Maintain visual sync with decimation (500 / 10 = 50Hz)
        device="cuda:0",
        gravity=(0.0, 0.0, -9.81), # Explicit Earth Gravity
        physx=sim_utils.PhysxCfg(
            solver_type=1, # TGS
            max_position_iteration_count=16, 
            max_velocity_iteration_count=4, 
            bounce_threshold_velocity=0.5, 
            enable_ccd=True, 
            enable_stabilization=True,
            gpu_max_rigid_contact_count=2**16, # 64k contacts
            gpu_max_rigid_patch_count=2**13, # 8k patches (increased from 2**11 due to patch buffer overflow)
            gpu_heap_capacity=2**26, # 64MB heap
            gpu_found_lost_pairs_capacity=2**13,
        ),
    )

    def __post_init__(self):
        self.decimation = 10 # 500Hz / 10 = 50Hz control (Nyquist stability for 6cm clearance)
        self.episode_length_s = 3000.0 
        self.viewer.camera_follow_prim_path = None
        # Enable cameras conditionally
        if not self.enable_cameras:
            self.scene.tiled_camera = None
            self.observations.policy.tiled_camera = None
