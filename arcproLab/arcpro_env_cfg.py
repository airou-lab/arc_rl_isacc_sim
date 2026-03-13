# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from isaaclab.utils import configclass
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.managers import ObservationGroupCfg, ObservationTermCfg as ObsTerm
from isaaclab.managers import ActionTermCfg as ActionTerm, RewardTermCfg as RewTerm, TerminationTermCfg as DoneTerm
from isaaclab.assets import AssetBaseCfg
from isaaclab.sensors import TiledCameraCfg, ContactSensorCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sim import SimulationCfg
import isaaclab.sim as sim_utils
import isaaclab.envs.mdp as mdp

from arcpro_robot_cfg import ARCPRO_ROBOT_CFG
import mdp.observations as mdp_obs
import mdp.rewards as mdp_rew
import mdp.terminations as mdp_done

@configclass
class ARCProSceneCfg(InteractiveSceneCfg):
    """Configuration for the ARCPro scene."""
    # ground plane
    ground = AssetBaseCfg(
        prim_path="/World/ground",
        spawn=sim_utils.MeshCuboidCfg(
            size=(100.0, 100.0, 0.001),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.1, 0.1, 0.1)),
        ),
    )
    # track (cloned per env)
    track = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Track",
        spawn=sim_utils.UsdFileCfg(
            usd_path="/home/arika/Documents/arcpro/arcpro_system/src/examples/ARCPro_RL/arc_rl_isacc_sim/openStreetUSD/arcpro_RL_open_street_sim.usd",
            scale=(1.0, 1.0, 1.0),
        ),
        init_state=AssetBaseCfg.InitialStateCfg(pos=(278.21, 200.52, 0.0)),
    )
    # robot
    robot = ARCPRO_ROBOT_CFG.replace(
        prim_path="{ENV_REGEX_NS}/Robot",
        init_state=ARCPRO_ROBOT_CFG.init_state.replace(pos=(0.0, 0.0, 0.05))
    )
    
    # per-agent camera (for policy or high-res monitoring)
    tiled_camera = TiledCameraCfg(
        prim_path="{ENV_REGEX_NS}/Robot/Rigid_Bodies/Chassis/Camera_Left",
        update_period=1.0/30.0,
        width=160,
        height=90,
        data_types=["rgb"],
        spawn=None, # Already exists in robot USD
    )

    # God View monitor disabled to save VRAM and fix transform order issues
    god_view = None

    # contact sensor
    contact_forces = ContactSensorCfg(
        prim_path="{ENV_REGEX_NS}/Robot/Rigid_Bodies/.*", 
        update_period=0.0, 
        history_length=3, 
        debug_vis=False,
        filter_prim_paths_expr=["{ENV_REGEX_NS}/Track/.*"],
    )

@configclass
class ObservationCfg:
    """Observation specifications for the environment."""
    @configclass
    class PolicyCfg(ObservationGroupCfg):
        """Telemetry group observations."""
        telemetry = ObsTerm(func=mdp_obs.get_telemetry_vector)
        
        def __post_init__(self):
            self.enable_corruption = False
            self.concatenate_terms = True

    @configclass
    class VisualCfg(ObservationGroupCfg):
        """Visual group observations."""
        # This will be populated dynamically in ARCProEnvCfg.__post_init__
        pass

    # observation groups
    policy: PolicyCfg = PolicyCfg()
    # visual group is added dynamically in ARCProEnvCfg.__post_init__ if enabled

@configclass
class ActionCfg:
    """Action specifications for the environment."""
    # Action index 0: Steering (Knuckles)
    steering = mdp.JointPositionActionCfg(
        asset_name="robot", 
        joint_names=["Knuckle__Upright__Front_Left", "Knuckle__Upright__Front_Right"],
    )
    # Action index 1: Throttle (Wheels)
    throttle = mdp.JointVelocityActionCfg(
        asset_name="robot", 
        joint_names=[
            "Wheel__Knuckle__Front_Left", 
            "Wheel__Knuckle__Front_Right", 
            "Wheel__Upright__Rear_Left", 
            "Wheel__Upright__Rear_Right"
        ],
        scale=40.0, # 1.0 action -> 40 rad/s
    )

@configclass
class RewardCfg:
    """Reward specifications for the environment."""
    speed = RewTerm(func=mdp_rew.speed_reward, weight=1.0)
    lateral_error = RewTerm(func=mdp_rew.lateral_error_reward, weight=1.0)
    line_penalty = RewTerm(func=mdp_rew.line_penalty, weight=1.0)
    jerk = RewTerm(func=mdp_rew.steering_jerk_penalty, weight=-0.1)

@configclass
class TerminationCfg:
    """Termination specifications for the environment."""
    white_line_contact = DoneTerm(func=mdp_done.white_line_contact)
    height = DoneTerm(func=mdp_done.height_termination)

from isaaclab.managers import SceneEntityCfg, ObservationGroupCfg, EventTermCfg as EventTerm
import mdp.events as mdp_event

@configclass
class EventCfg:
    """Event specifications for the environment."""
    # Stability pass (set damping/stiffness)
    stability_pass = EventTerm(
        func=mdp_event.setup_robot_stability,
        mode="startup",
        params={"asset_cfg": SceneEntityCfg("robot")},
    )

@configclass
class ARCProEnvCfg(ManagerBasedRLEnvCfg):
    """Configuration for the ARCPro environment."""
    # Custom flags
    enable_cameras: bool = False

    # Scene settings
    scene: ARCProSceneCfg = ARCProSceneCfg(num_envs=128, env_spacing=5.0)
    # Observation settings
    observations: ObservationCfg = ObservationCfg()
    # Action settings
    actions: ActionCfg = ActionCfg()
    # Event settings
    events: EventCfg = EventCfg()
    # Reward settings
    rewards: RewardCfg = RewardCfg()
    # Termination settings
    terminations: TerminationCfg = TerminationCfg()
    
    # Simulation settings
    sim: SimulationCfg = SimulationCfg(
        dt=1.0/60.0,
        render_interval=2, # 30Hz rendering
        device="cuda:0",
    )

    def __post_init__(self):
        """Post-initialization of the configuration."""
        self.decimation = 2 # 30Hz control
        self.episode_length_s = 30.0 # 30s max episode

        # Handle camera soft-disable
        if not self.enable_cameras:
            # Disable per-agent tiled cameras to save VRAM
            self.scene.tiled_camera = None
        else:
            # Add to visual observations if enabled
            from isaaclab.managers import SceneEntityCfg, ObservationGroupCfg
            self.observations.visual = ObservationGroupCfg()
            self.observations.visual.tiled_camera = ObsTerm(
                func=mdp.image_uint8, 
                params={"sensor_cfg": SceneEntityCfg("tiled_camera")}
            )
        
        # Always keep god_view monitor (attached to env_0)
        # Note: We don't add it to observations to avoid policy dependency, 
        # but it remains in the scene for visual monitoring if not explicitly set to None.
