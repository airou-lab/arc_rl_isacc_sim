# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import os
import sys

sys.path.append(os.path.dirname(os.path.abspath(__file__)))

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

@configclass
class EventCfg:
    """Configuration for events."""

    reset_robot_to_lane = EventTermCfg(
        func=mdp_events.reset_robot_to_lane,
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

    # Track from no_graph_sim_final.usd (Stable grounded track from main branch)
    track = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Track",
        spawn=sim_utils.UsdFileCfg(
            usd_path=os.path.join(os.path.dirname(__file__), "..", "openStreetUSD", "no_graph_sim_final.usd"),
            collision_props=sim_utils.CollisionPropertiesCfg(collision_enabled=True),
            scale=(0.0825, 0.0825, 0.0825), 
        ),
        init_state=AssetBaseCfg.InitialStateCfg(pos=(0.0, 0.0, -1.25)),
    )

    
    # Robot (1.0x Metric Scale, 0.5m drop above the track)
    robot = ARCPRO_ROBOT_CFG.replace(
        prim_path="{ENV_REGEX_NS}/Robot",
        spawn=ARCPRO_ROBOT_CFG.spawn.replace(
            scale=(1.0, 1.0, 1.0),
        ),
        init_state=ARCPRO_ROBOT_CFG.init_state.replace(pos=(0.0, 0.0, 0.5)), 
    )
    
    # Camera
    tiled_camera = TiledCameraCfg(
        prim_path="{ENV_REGEX_NS}/Robot/Chassis/CameraSensor",
        update_period=0.0,
        spawn=sim_utils.PinholeCameraCfg(),
        offset=TiledCameraCfg.OffsetCfg(pos=(0.28, 0.0, 0.16), rot=(1.0, 0.0, 0.0, 0.0), convention="parent"),
        data_types=["rgb"], width=160, height=90,
    )

    # Contact Sensor
    contact_forces = ContactSensorCfg(
        prim_path="{ENV_REGEX_NS}/Robot/.*", 
        update_period=0.0, 
        history_length=3, 
        debug_vis=False,
        filter_prim_paths_expr=["{ENV_REGEX_NS}/Track/.*"]
    )

@configclass
class ObservationCfg:
    @configclass
    class PolicyCfg(ObservationGroupCfg):
        telemetry = ObsTerm(func=mdp_obs.get_telemetry_vector)
    policy: PolicyCfg = PolicyCfg()
    
    @configclass
    class VisualCfg(ObservationGroupCfg):
        tiled_camera = ObsTerm(func=mdp.image, params={"sensor_cfg": SceneEntityCfg("tiled_camera"), "normalize": False})
    visual: VisualCfg = VisualCfg()

@configclass
class ActionCfg:
    steering = mdp.JointPositionActionCfg(asset_name="robot", joint_names=["Joint_Steer_L", "Joint_Steer_R"], scale=1.0, preserve_order=True)
    throttle = mdp.JointVelocityActionCfg(asset_name="robot", joint_names=["Joint_Drive_.*"], scale=1.0, preserve_order=True)

@configclass
class RewardCfg:
    speed = RewTerm(func=mdp_rew.speed_reward, weight=1.0)
    lateral_error = RewTerm(func=mdp_rew.lateral_error_reward, weight=1.0)
    collision = RewTerm(func=mdp_rew.collision_penalty, weight=1.0)

@configclass
class TerminationCfg:
    # height = DoneTerm(func=mdp_done.height_termination)
    white_line_contact = DoneTerm(func=mdp_done.white_line_contact)
    base_contact = DoneTerm(func=mdp_done.white_line_contact)

@configclass
class ARCProEnvCfg(ManagerBasedRLEnvCfg):
    # Adjust viewer to see the scene at ground-level
    viewer: ViewerCfg = ViewerCfg(eye=(15.0, 15.0, 15.0), lookat=(0.0, 0.0, 0.0))
    
    enable_cameras: bool = True
    scene: ARCProSceneCfg = ARCProSceneCfg(num_envs=1, env_spacing=1000.0)
    observations: ObservationCfg = ObservationCfg()
    actions: ActionCfg = ActionCfg()
    rewards: RewardCfg = RewardCfg()
    terminations: TerminationCfg = TerminationCfg()
    events: EventCfg = EventCfg()

    sim: SimulationCfg = SimulationCfg(
        dt=0.005, # 200Hz for balanced performance
        render_interval=8, # Maintain 25Hz visual (200 / 8)
        device="cuda:0",
        physx=sim_utils.PhysxCfg(
            solver_type=1, # TGS
            max_position_iteration_count=8, # From main branch
            max_velocity_iteration_count=4, # From main branch
            bounce_threshold_velocity=0.5, 
            enable_ccd=True, 
            enable_stabilization=True,
            gpu_max_rigid_contact_count=2**21,
            gpu_max_rigid_patch_count=2**18,
            gpu_heap_capacity=2**26,
            gpu_found_lost_pairs_capacity=2**21,
        ),
    )

    def __post_init__(self):
        self.decimation = 8 # Sync with render_interval
        self.episode_length_s = 120.0 
        self.viewer.camera_follow_prim_path = None
        
        if not self.enable_cameras:
            self.observations.visual = None
            self.scene.tiled_camera = None
