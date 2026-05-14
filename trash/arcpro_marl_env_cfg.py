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
import mdp.actions as arcpro_actions

##
# Multi-Agent Configuration
##
NUM_AGENTS = 4

@configclass
class ARCProMARLSceneCfg(InteractiveSceneCfg):
    """MARL Prototype: 4 Robots in a shared 1.0x Intersection."""
    
    # Lighting
    light = AssetBaseCfg(
        prim_path="/World/light", 
        spawn=sim_utils.DistantLightCfg(intensity=3000.0, color=(1.0, 1.0, 1.0))
    )

    # Ground Plane
    ground_plane = AssetBaseCfg(
        prim_path="/World/defaultGroundPlane",
        spawn=sim_utils.GroundPlaneCfg(),
        init_state=AssetBaseCfg.InitialStateCfg(pos=(0.0, 0.0, -0.05)),
    )

    # Track
    track = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Track",
        spawn=sim_utils.UsdFileCfg(
            usd_path=os.path.join(USD_DIR, "no_graph_sim_clean_1x_flattened.usda"),
            scale=(1.0, 1.0, 1.0),
        ),
        init_state=AssetBaseCfg.InitialStateCfg(pos=(0.0, 0.0, 0.0)),
    )

    # Procedural Robot Spawning
    # North, South, East, West entry points relative to the main intersection
    spawn_points = [
        (-16.25, 5.65, 0.10, 0.7071, 0.0, 0.0, -0.7071),  # North (Facing South)
        (-16.25, -5.00, 0.10, 0.7071, 0.0, 0.0, 0.7071),  # South (Facing North)
        (-10.00, 0.35, 0.10, 1.0, 0.0, 0.0, 0.0),         # East (Facing West)
        (-22.50, 0.35, 0.10, 0.0, 0.0, 0.0, 1.0),         # West (Facing East)
    ]

    def __post_init__(self):
        # We define robots here instead of as class attributes to support NUM_AGENTS
        for i in range(NUM_AGENTS):
            pos_x, pos_y, pos_z, qw, qx, qy, qz = self.spawn_points[i % 4]
            # Adjust position slightly if more than 4 agents to prevent stacking
            if i >= 4:
                pos_y += (i // 4) * 2.0 
            
            robot_cfg = ARCPRO_ROBOT_CFG.replace(
                prim_path=f"{{ENV_REGEX_NS}}/Robot_{i}",
                spawn=ARCPRO_ROBOT_CFG.spawn.replace(
                    usd_path=os.path.join(ARCPRO_LAB_DIR, "assets", "robot", "F1Tenth_Metric.usd"),
                ),
                init_state=ARCPRO_ROBOT_CFG.init_state.replace(
                    pos=(pos_x, pos_y, pos_z),
                    rot=(qw, qx, qy, qz)
                ),
            )
            setattr(self, f"robot_{i}", robot_cfg)
            
            # Individual cameras per robot
            camera_cfg = TiledCameraCfg(
                prim_path=f"{{ENV_REGEX_NS}}/Robot_{i}/Chassis/CameraSensor",
                update_period=0.0,
                spawn=sim_utils.PinholeCameraCfg(horizontal_aperture=2.65, focal_length=1.93),
                offset=TiledCameraCfg.OffsetCfg(pos=(0.4, 0.0, 0.2), rot=(0.985, 0.0, 0.174, 0.0), convention="parent"),
                data_types=["rgb"], width=160, height=90,
            )
            setattr(self, f"tiled_camera_{i}", camera_cfg)

            # Individual contact sensors
            contact_cfg = ContactSensorCfg(
                prim_path=f"{{ENV_REGEX_NS}}/Robot_{i}/Chassis",
                update_period=0.0,
                history_length=3,
            )
            setattr(self, f"contact_forces_{i}", contact_cfg)

@configclass
class ActionCfg:
    """Procedural Action Management for N agents."""
    def __init__(self):
        for i in range(NUM_AGENTS):
            setattr(self, f"steering_{i}", arcpro_actions.AckermannSteeringActionCfg(
                asset_name=f"robot_{i}",
                joint_names=["Joint_Steer_L", "Joint_Steer_R"], 
                wheelbase=0.33, track_width=0.28, max_steering_angle=0.5
            ))
            setattr(self, f"drive_{i}", arcpro_actions.CombinedDriveActionCfg(
                asset_name=f"robot_{i}", 
                joint_names=["Joint_Drive_RL", "Joint_Drive_RR", "Joint_Drive_FL", "Joint_Drive_FR"], 
                scale=15.0
            ))

@configclass
class ObservationCfg:
    @configclass
    class PolicyCfg(ObservationGroupCfg):
        def __init__(self):
            super().__init__()
            for i in range(NUM_AGENTS):
                # Standard Telemetry (Zero-Loc)
                setattr(self, f"telemetry_{i}", ObsTerm(
                    func=mdp_obs.get_telemetry_vector, 
                    params={"asset_cfg": SceneEntityCfg(f"robot_{i}")}
                ))
                # New V2V Observation: Includes the Peer-to-Peer Go Signal
                setattr(self, f"v2v_{i}", ObsTerm(
                    func=mdp_obs.get_v2v_observation, 
                    params={"robot_index": i}
                ))
    policy: PolicyCfg = PolicyCfg()

@configclass
class RewardCfg:
    """Procedural Reward Management for N agents."""
    def __init__(self):
        for i in range(NUM_AGENTS):
            # Anti-Suicide: Heavy penalty for crashing/resetting
            setattr(self, f"terminating_{i}", RewTerm(
                func=mdp_rew.termination_penalty, 
                weight=1.0
            ))
            
            setattr(self, f"speed_{i}", RewTerm(
                func=mdp_rew.speed_reward, 
                weight=20.0, 
                params={"asset_cfg": SceneEntityCfg(f"robot_{i}")}
            ))
            
            setattr(self, f"lateral_error_{i}", RewTerm(
                func=mdp_rew.lateral_error_reward, 
                weight=5.0,
                params={"asset_cfg": SceneEntityCfg(f"robot_{i}")}
            ))
            
            setattr(self, f"smoothness_{i}", RewTerm(
                func=mdp_rew.action_rate_smoothness_reward, 
                weight=10.0,
                params={"asset_cfg": SceneEntityCfg(f"robot_{i}")}
            ))

            # V2V Adherence: Penalty for jumping the queue
            setattr(self, f"queue_jumping_{i}", RewTerm(
                func=mdp_rew.queue_jumping_penalty, 
                weight=1.0,
                params={"robot_index": i}
            ))

@configclass
class TerminationCfg:
    # Collision detection: Reset if any two robots hit
    robot_collision = DoneTerm(func=mdp_done.robot_robot_collision)
    
    # Individual boundary hits
    roadmark_contact = DoneTerm(func=mdp_done.white_line_contact, params={"asset_cfg": SceneEntityCfg("robot_0")})

    # Individual Queue Jumping
    def __init__(self):
        for i in range(NUM_AGENTS):
            setattr(self, f"queue_jump_{i}", DoneTerm(
                func=mdp_done.queue_jumping_termination, 
                params={"robot_index": i}
            ))

@configclass
class ARCProMARLEnvCfg(ManagerBasedRLEnvCfg):
    viewer: ViewerCfg = ViewerCfg(eye=(-15.0, 6.875, 1.25), lookat=(-16.25, 5.56, 0.0))
    
    scene: ARCProMARLSceneCfg = ARCProMARLSceneCfg(num_envs=1, env_spacing=50.0)
    observations: ObservationCfg = ObservationCfg()
    actions: ActionCfg = ActionCfg()
    rewards: RewardCfg = RewardCfg()
    terminations: TerminationCfg = TerminationCfg()
    
    sim: SimulationCfg = SimulationCfg(
        dt=0.002, render_interval=25,
        physx=sim_utils.PhysxCfg(solver_type=1, max_position_iteration_count=16),
    )

    def __post_init__(self):
        self.decimation = 25
        self.episode_length_s = 60.0 
