# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import os
import sys

sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from isaaclab.utils import configclass
from isaaclab.envs import ManagerBasedRLEnvCfg, ViewerCfg
from isaaclab.managers import ObservationGroupCfg, ObservationTermCfg as ObsTerm, ActionTermCfg as ActionTerm, RewardTermCfg as RewTerm, TerminationTermCfg as DoneTerm, SceneEntityCfg
from isaaclab.assets import AssetBaseCfg
from isaaclab.sensors import CameraCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sim import SimulationCfg
import isaaclab.sim as sim_utils
import isaaclab.envs.mdp as mdp

from arcpro_robot_cfg import ARCPRO_ROBOT_CFG
import mdp.observations as mdp_obs, mdp.rewards as mdp_rew, mdp.terminations as mdp_done

@configclass
class ARCProSceneCfg(InteractiveSceneCfg):
    """GSD Phase 7: Single-Surface Mode (20.0x Scale)."""
    
    light = AssetBaseCfg(
        prim_path="/World/light", 
        spawn=sim_utils.DistantLightCfg(intensity=3000.0, color=(1.0, 1.0, 1.0))
    )

    # NO GROUND PLANE - preventing collision conflicts
    
    track = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Track",
        spawn=sim_utils.UsdFileCfg(
            usd_path=os.path.join(os.path.dirname(__file__), "..", "openStreetUSD", "no_graph_sim_cleaned.usd"),
            collision_props=sim_utils.CollisionPropertiesCfg(collision_enabled=True),
        ),
        init_state=AssetBaseCfg.InitialStateCfg(pos=(0.0, 0.0, 0.0)),
    )
    
    # ROBOT SCALE: 20.0x
    robot = ARCPRO_ROBOT_CFG.replace(
        prim_path="{ENV_REGEX_NS}/Robot",
        spawn=ARCPRO_ROBOT_CFG.spawn.replace(
            scale=(20.0, 20.0, 20.0),
            # Increase offsets for massive parts
            collision_props=sim_utils.CollisionPropertiesCfg(
                collision_enabled=True,
                contact_offset=0.05,
                rest_offset=0.0,
            ),
        ),
        init_state=ARCPRO_ROBOT_CFG.init_state.replace(
            pos=(0.0, 0.0, 20.0) # Drop from 20m
        )
    )
    
    # Camera
    tiled_camera = CameraCfg(
        prim_path="{ENV_REGEX_NS}/Robot/Chassis/CameraSensor",
        update_period=0.0,
        spawn=sim_utils.PinholeCameraCfg(),
        offset=CameraCfg.OffsetCfg(pos=(2.8, 0.0, 1.6), rot=(1.0, 0.0, 0.0, 0.0), convention="world"),
        data_types=["rgb"], width=160, height=90,
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
    steering = mdp.JointPositionActionCfg(asset_name="robot", joint_names=["Joint_Steer_.*"], scale=1.0, preserve_order=True)
    throttle = mdp.JointVelocityActionCfg(asset_name="robot", joint_names=["Joint_Drive_.*"], scale=1.0, preserve_order=True)

@configclass
class RewardCfg:
    speed = RewTerm(func=mdp_rew.speed_reward, weight=1.0)

@configclass
class TerminationCfg:
    height = DoneTerm(func=mdp_done.height_termination)

@configclass
class ARCProEnvCfg(ManagerBasedRLEnvCfg):
    viewer: ViewerCfg = ViewerCfg(eye=(50.0, 50.0, 50.0), lookat=(0.0, 0.0, 0.0))
    
    enable_cameras: bool = True
    scene: ARCProSceneCfg = ARCProSceneCfg(num_envs=1, env_spacing=1000.0)
    observations: ObservationCfg = ObservationCfg()
    actions: ActionCfg = ActionCfg()
    rewards: RewardCfg = RewardCfg()
    terminations: TerminationCfg = TerminationCfg()

    sim: SimulationCfg = SimulationCfg(
        dt=0.005, render_interval=8, device="cuda:0",
        physx=sim_utils.PhysxCfg(
            solver_type=1,
            max_position_iteration_count=8, 
            max_velocity_iteration_count=4,
            bounce_threshold_velocity=1.0, 
            enable_ccd=False, 
            enable_stabilization=True,
        ),
    )

    def __post_init__(self):
        self.decimation = 8
        self.episode_length_s = 120.0 
        self.viewer.camera_follow_prim_path = None
        
        if not self.enable_cameras:
            self.observations.visual = None
            self.scene.tiled_camera = None
