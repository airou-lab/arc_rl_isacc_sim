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
    # REMOVED: User wants robot to fall into void
    # ground_plane = AssetBaseCfg(
    #     prim_path="/World/defaultGroundPlane",
    #     spawn=sim_utils.GroundPlaneCfg(),
    #     init_state=AssetBaseCfg.InitialStateCfg(pos=(0.0, 0.0, -0.1)),
    # )

    # Track from no_graph_sim.usd (Clean 1x version: No grass/foliage/fences)
    track = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Track",
        spawn=sim_utils.UsdFileCfg(
            usd_path=os.path.join(USD_DIR, "no_graph_sim_clean_1x.usda"),
            scale=(0.125, 0.125, 0.125), # Shrink world to match 1.0x robot
        ),
        # Use origin position to match USD world coordinates
        init_state=AssetBaseCfg.InitialStateCfg(pos=(0.0, 0.0, 0.0)),
    )


    
    # Robot (1.0x Metric Scale)
    robot = ARCPRO_ROBOT_CFG.replace(
        prim_path="{ENV_REGEX_NS}/Robot",
        spawn=ARCPRO_ROBOT_CFG.spawn.replace(
            usd_path=os.path.join(ARCPRO_LAB_DIR, "assets", "robot", "F1Tenth_Metric.usd"),
            scale=(1.0, 1.0, 1.0), # Revert to 1.0x
        ),
        init_state=ARCPRO_ROBOT_CFG.init_state.replace(
            # Spawn point: right lane heading toward junction_18 intersection.
            # Y/Z and yaw unchanged from prior centerline spawn; X shifted from
            # -16.25375 -> -15.73 (a ~+0.52 m offset) to center the body in the
            # right lane. Verified visually via inspect_world.py iterating
            # -15.0 (off-road) -> -15.75 -> -15.70 -> -15.73.
            pos=(-15.73, 5.56, 0.05),
            rot=(0.7071, 0.0, 0.0, 0.7071) # +90 degrees Z-up (faces +Y)
        ),
    )
    
    # Camera (Standard offset for 1x robot)
    tiled_camera = TiledCameraCfg(
        prim_path="{ENV_REGEX_NS}/Robot/Chassis/CameraSensor",
        update_period=0.0,
        spawn=sim_utils.PinholeCameraCfg(
            horizontal_aperture=2.65, # Approx 69 deg FOV
            focal_length=1.93,
        ),
        # 8x pos: (2.24, 0.0, 1.28) -> 1x pos: (0.28, 0.0, 0.16)
        offset=TiledCameraCfg.OffsetCfg(pos=(0.28, 0.0, 0.16), rot=(1.0, 0.0, 0.0, 0.0), convention="parent"),
        data_types=["rgb"], width=224, height=224,
    )

    # Camera Visual Helper (Invisible to sensors, visible in GUI)
    camera_cone = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Robot/Chassis/CameraVisualCone",
        spawn=sim_utils.ConeCfg(
            func=arcpro_spawner.spawn_guide_cone,
            radius=0.03,
            height=0.1,
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.0, 1.0, 1.0), opacity=0.5),
        ),
        init_state=AssetBaseCfg.InitialStateCfg(
            pos=(0.35, 0.0, 0.16), # Slightly in front of camera lens
            rot=(0.7071, 0.0, 0.7071, 0.0) # Point along X-axis (Forward)
        ),
    )

    # Contact Sensor: Detect chassis collisions (crashes)
    contact_forces = ContactSensorCfg(
        prim_path="{ENV_REGEX_NS}/Robot/Chassis",
        update_period=0.0,
        history_length=3,
        debug_vis=True,
    )

@configclass
class ObservationCfg:
    @configclass
    class PolicyCfg(ObservationGroupCfg):
        # concatenate_terms=False keeps telemetry (1-D) and tiled_camera (3-D)
        # as separate keys in the obs dict; IsaacLab would otherwise try to
        # torch.stack their shape descriptors and fail. SB3's MultiInputPolicy
        # consumes a Dict space natively (CNN extractor on image, MLP on vec).
        concatenate_terms = False
        telemetry = ObsTerm(func=mdp_obs.get_telemetry_vector)
        tiled_camera = ObsTerm(
            func=mdp_obs.get_image_uint8,
            params={"sensor_cfg": SceneEntityCfg("tiled_camera")},
            # clip sets the gym.spaces.Box low/high used by SB3's image-space
            # detection (which requires bounds of either [-1,1] or [0,255]).
            # Integer literals so torch.clip_ on the uint8 tensor doesn't try
            # to cast float bounds to uint8 in-place (errors out).
            clip=(0, 255),
        )

    policy: PolicyCfg = PolicyCfg()

import mdp.actions as arcpro_actions

@configclass
class ActionCfg:
    steering = arcpro_actions.GroupedJointPositionActionCfg(asset_name="robot", joint_names=["Joint_Steer_L", "Joint_Steer_R"], scale=1.0)
    # Throttle + brake folded into one 2-D term. Outer action vector remains
    # [steer (1), throttle (1), brake (1)] = 3 channels, matching the policy
    # repo's (steer, throttle, brake) contract. Brake derates throttle:
    # wheel_vel = 60.0 * throttle * (1 - brake), same as policy's isaac_direct_env.
    throttle = arcpro_actions.ThrottleBrakeVelocityActionCfg(
        asset_name="robot",
        joint_names=["Joint_Drive_RL", "Joint_Drive_RR", "Joint_Drive_FL", "Joint_Drive_FR"],
        scale=60.0,
    )

@configclass
class RewardCfg:
    speed = RewTerm(func=mdp_rew.speed_reward, weight=5.0) # Increased from 1.0
    # Moderate lateral error penalty (Speed reward now competitive with staying on track)
    lateral_error = RewTerm(func=mdp_rew.lateral_error_reward, weight=5.0)
    # Discourage staying still (increased to prevent creeping)
    stationary = RewTerm(
        func=lambda env: torch.where(env.scene["robot"].data.root_lin_vel_b[:, 0] < 0.5, -5.0, 0.0),
        weight=1.0
    )
    # Prevent 180s
    heading = RewTerm(func=mdp_rew.heading_alignment_reward, weight=2.0)
    # Smoothness Calibration: Penalize steering wiggles
    smoothness = RewTerm(func=mdp_rew.action_rate_smoothness_reward, weight=1.0)

@configclass
class TerminationCfg:
    # height termination: Catch flying robots
    height = DoneTerm(func=mdp_done.height_termination)
    # Contact termination: Reset if hitting roadmarks (white lines)
    roadmark_contact = DoneTerm(func=mdp_done.white_line_contact)
    # Stagnation: Reset if stuck against a wall
    stagnation = DoneTerm(func=mdp_done.stagnation_termination)
    # FOV Driving: Reset if driving into areas not visible to the camera
    driving_blind = DoneTerm(
        func=mdp_done.fov_visibility_termination,
        params={"horizontal_aperture": 2.65, "focal_length": 1.93}
    )

@configclass
class ARCProEnvCfg(ManagerBasedRLEnvCfg):
    # Adjust viewer to see the robot at its new world position
    # 8x eye: (-120.0, 55.0, 10.0) -> 1x eye: (-15.0, 6.875, 1.25)
    viewer: ViewerCfg = ViewerCfg(eye=(-15.0, 6.875, 1.25), lookat=(-16.25, 5.56, 0.0))
    
    enable_cameras: bool = True
    scene: ARCProSceneCfg = ARCProSceneCfg(num_envs=32, env_spacing=50.0)
    observations: ObservationCfg = ObservationCfg()
    actions: ActionCfg = ActionCfg()
    rewards: RewardCfg = RewardCfg()
    terminations: TerminationCfg = TerminationCfg()
    events: EventCfg = EventCfg()
    sim: SimulationCfg = SimulationCfg(
        dt=0.002, # 500Hz for high-fidelity small scale physics
        render_interval=25, # Maintain 20Hz visual (500 / 25)
        device="cuda:0",
        physx=sim_utils.PhysxCfg(
            solver_type=1, # TGS
            max_position_iteration_count=16, # Increased from 8
            max_velocity_iteration_count=4, 
            bounce_threshold_velocity=0.5, 
            enable_ccd=True, 
            enable_stabilization=True,
            gpu_max_rigid_contact_count=2**20, # 1M contacts
            gpu_max_rigid_patch_count=2**17,
            gpu_heap_capacity=2**26, # 64MB heap
            gpu_found_lost_pairs_capacity=2**20,
        ),
    )

    def __post_init__(self):
        self.decimation = 25 # Sync with render_interval (500Hz / 25 = 20Hz control)
        self.episode_length_s = 120.0 
        self.viewer.camera_follow_prim_path = None
        
        if not self.enable_cameras:
            self.observations.policy.tiled_camera = None
            self.scene.tiled_camera = None
