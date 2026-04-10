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
        init_state=AssetBaseCfg.InitialStateCfg(pos=(0.0, 0.0, -1.0)),
    )

    # Track from no_graph_sim.usd (Original visuals, hardened in-place)
    track = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Track",
        spawn=sim_utils.UsdFileCfg(
            usd_path=os.path.join(USD_DIR, "no_graph_sim.usd"),
            scale=(1.0, 1.0, 1.0), 
        ),
        # Use origin position to match USD world coordinates
        init_state=AssetBaseCfg.InitialStateCfg(pos=(0.0, 0.0, 0.0)),
    )


    
    # Robot (8.0x Metric Scale based on GUI resize)
    robot = ARCPRO_ROBOT_CFG.replace(
        prim_path="{ENV_REGEX_NS}/Robot",
        spawn=ARCPRO_ROBOT_CFG.spawn.replace(
            usd_path=os.path.join(ARCPRO_LAB_DIR, "assets", "robot", "F1Tenth_Metric.usd"),
            scale=(8.0, 8.0, 8.0),
        ),
        init_state=ARCPRO_ROBOT_CFG.init_state.replace(
            # Fixed Spawn Point: Centerline (Shifted to align with Waypoints)
            pos=(-130.03, 44.48, 0.42), 
            rot=(0.7071, 0.0, 0.0, 0.7071) # +90 degrees Z-up (Flipped 180)
        ), 
    )
    
    # Camera (Scaled offset for 8x robot)
    tiled_camera = TiledCameraCfg(
        prim_path="{ENV_REGEX_NS}/Robot/Chassis/CameraSensor",
        update_period=0.0,
        spawn=sim_utils.PinholeCameraCfg(),
        offset=TiledCameraCfg.OffsetCfg(pos=(2.24, 0.0, 1.28), rot=(1.0, 0.0, 0.0, 0.0), convention="parent"),
        data_types=["rgb"], width=160, height=90,
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
        telemetry = ObsTerm(func=mdp_obs.get_telemetry_vector)
    policy: PolicyCfg = PolicyCfg()
    
    @configclass
    class VisualCfg(ObservationGroupCfg):
        tiled_camera = ObsTerm(func=mdp.image, params={"sensor_cfg": SceneEntityCfg("tiled_camera"), "normalize": True})
    
    visual: VisualCfg | None = VisualCfg()

import mdp.actions as arcpro_actions

@configclass
class ActionCfg:
    steering = arcpro_actions.GroupedJointPositionActionCfg(asset_name="robot", joint_names=["Joint_Steer_L", "Joint_Steer_R"], scale=1.0)
    throttle = arcpro_actions.GroupedJointVelocityActionCfg(asset_name="robot", joint_names=["Joint_Drive_RL", "Joint_Drive_RR", "Joint_Drive_FL", "Joint_Drive_FR"], scale=40.0)

@configclass
class RewardCfg:
    speed = RewTerm(func=mdp_rew.speed_reward, weight=1.0)
    # Strong lateral error penalty (Off-track penalty > Speed reward)
    lateral_error = RewTerm(func=mdp_rew.lateral_error_reward, weight=5.0)
    # Discourage staying still
    stationary = RewTerm(
        func=lambda env: torch.where(env.scene["robot"].data.root_lin_vel_b[:, 0] < 0.1, -1.0, 0.0),
        weight=1.0
    )

@configclass
class TerminationCfg:
    # height termination: Catch flying robots
    height = DoneTerm(func=mdp_done.height_termination)
    # Contact termination: Reset if hitting roadmarks (white lines)
    # Balanced at 2.2m (0.275 normalized) for 8x robot clearance
    roadmark_contact = DoneTerm(func=mdp_done.white_line_contact)

@configclass
class ARCProEnvCfg(ManagerBasedRLEnvCfg):
    # Adjust viewer to see the robot at its new world position
    viewer: ViewerCfg = ViewerCfg(eye=(-120.0, 55.0, 10.0), lookat=(-129.0, 46.0, 0.0))
    
    enable_cameras: bool = True
    scene: ARCProSceneCfg = ARCProSceneCfg(num_envs=1, env_spacing=1000.0)
    observations: ObservationCfg = ObservationCfg()
    actions: ActionCfg = ActionCfg()
    rewards: RewardCfg = RewardCfg()
    terminations: TerminationCfg = TerminationCfg()
    events: EventCfg = EventCfg()

    sim: SimulationCfg = SimulationCfg(
        dt=0.005, # 200Hz for balanced performance
        render_interval=10, # Maintain 20Hz visual (200 / 10)
        device="cuda:0",
        physx=sim_utils.PhysxCfg(
            solver_type=1, # TGS
            max_position_iteration_count=8, # From main branch
            max_velocity_iteration_count=4, # From main branch
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
        self.decimation = 10 # Sync with render_interval
        self.episode_length_s = 120.0 
        self.viewer.camera_follow_prim_path = None
        
        if not self.enable_cameras:
            self.observations.visual = None
            self.scene.tiled_camera = None
