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
    
    # Lighting
    light = AssetBaseCfg(
        prim_path="/World/light", 
        spawn=sim_utils.DistantLightCfg(intensity=3000.0, color=(1.0, 1.0, 1.0))
    )

    # Ground Plane (Visual + Safety Backup)
    ground_plane = AssetBaseCfg(
        prim_path="/World/defaultGroundPlane",
        spawn=sim_utils.GroundPlaneCfg(),
        init_state=AssetBaseCfg.InitialStateCfg(pos=(0.0, 0.0, -0.05)),
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
            rot=(0.7071, 0.0, 0.0, 0.7071), # Upright & Face North (WXYZ)
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
        # Raised to 0.35m with no tilt to see further down the track
        offset=TiledCameraCfg.OffsetCfg(pos=(-0.3, 0.0, 0.35), rot=(1.0, 0.0, 0.0, 0.0), convention="parent"),
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
            pos=(-0.3, 0.0, 0.35),
            rot=(0.0, -0.866, 0.0, 0.5) # Points X forward correctly based on chassis
        )
    )

    # Contact Sensor: Detect chassis collisions (crashes)
    contact_forces = ContactSensorCfg(
        prim_path="{ENV_REGEX_NS}/Robot/Chassis",
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
        scale=-20.0, 
        offset=-20.0
    )

@configclass
class RewardCfg:
    # Survival Bonus: Reduced to 1.0. We rely mostly on bounded progress reward now.
    survival_bonus = RewTerm(func=lambda env: torch.ones(env.num_envs, device=env.device), weight=1.0)

    # Anti-Suicide: Reduced from extreme 100.0 (-5000) to 20.0 (-1000) to allow value function to learn smoothly.
    termination_penalty = RewTerm(func=mdp_rew.termination_penalty, weight=20.0)
    
    # Fix E: Bound the waypoint progress reward with tanh and boost weight to 50.0.
    # At 0.5 m/s (~1.7 WPs), tanh(1.7) = 0.93 * 50 = ~46 pts/step.
    # At 1.5 m/s (~5.1 WPs), tanh(5.1) = 0.99 * 50 = ~49 pts/step.
    # This massive reward bootstraps the learning of cornering (surviving 500 steps = 23k points),
    # but the tanh bound prevents the endless runner exploit because sprinting 3x faster only yields 6% more points!
    progress_reward = RewTerm(func=lambda env: torch.tanh(mdp_rew.waypoint_progress_reward(env)), weight=50.0)
    
    # Exploration: Tiny reward for applying any throttle/drive action to break out of stagnation
    action_drive_reward = RewTerm(
        func=lambda env: torch.abs(env.action_manager.action[:, 1]),
        weight=0.5
    )
    
    # Precision: Lane centering (Phase 2 constraint - currently disabled for Phase 1 curriculum)
    lateral_error = RewTerm(func=mdp_rew.lateral_error_reward, weight=0.0)
    
    # Force movement: Penalty forces it to move!
    # Fix Bug 5: Use torch.abs() so reverse driving doesn't spuriously trigger penalty.
    # Requires min speed of 0.5 m/s, UNLESS the traffic light (go_signal) says to stop!
    stationary = RewTerm(
        func=lambda env: torch.where(
            torch.abs(env.scene["robot"].data.root_lin_vel_b[:, 0]) < 0.2,
            -1.0, 0.0
        ),
        weight=50.0
    )
    
    # Phase 2: Waypoint Tracking (Disabled for Phase 1)
    heading = RewTerm(func=mdp_rew.heading_alignment_reward, weight=0.0)
    smoothness = RewTerm(func=mdp_rew.action_rate_smoothness_reward, weight=0.0)
    
    # Jitter Suppression (Disabled for Phase 1)
    jerk = RewTerm(func=mdp_rew.jerk_penalty, weight=0.0)
    
    # Boundary Penalty: (Enabled: Risk-aware shaping to smoothly steer away from walls)
    # Note: Native func returns -100.0. We use weight=0.4 so the penalty is -40.0.
    # This ensures a net positive score (+50 progress - 40 penalty = +10) in the warning track,
    # preventing 'Penalty Over-Saturation' for a newborn vision agent while still preferring the center.
    boundary = RewTerm(func=mdp_rew.boundary_penalty, weight=0.4)


@configclass
class TerminationCfg:
    roadmark_contact = DoneTerm(func=mdp_done.white_line_contact, params={"threshold": 0.12})
    # height termination: Catch flying robots (falling into void)
    # Lowered to 0.02m to give suspension room to breathe upon impact
    height = DoneTerm(func=mdp_done.height_termination, params={"threshold": 0.005})
    # Stagnation: Reset if stuck against a wall
    stagnation = DoneTerm(func=mdp_done.stagnation_termination)
    # FOV Driving: Reset if driving into areas not visible to the camera
    # driving_blind = DoneTerm(
    #     func=mdp_done.fov_visibility_termination,
    #     params={"horizontal_aperture": 3.86, "focal_length": 1.93}
    # )

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
            gpu_max_rigid_patch_count=2**11,
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
