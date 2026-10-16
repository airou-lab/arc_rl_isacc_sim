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
from isaaclab.managers import (
    ObservationGroupCfg,
    ObservationTermCfg as ObsTerm,
    RewardTermCfg as RewTerm,
    TerminationTermCfg as DoneTerm,
    SceneEntityCfg,
    EventTermCfg,
)
from isaaclab.assets import AssetBaseCfg
from isaaclab.sensors import TiledCameraCfg, ContactSensorCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sim import SimulationCfg
import isaaclab.sim as sim_utils

from arcpro_robot_cfg import ARCPRO_ROBOT_CFG
import mdp.observations as mdp_obs, mdp.rewards as mdp_rew, mdp.terminations as mdp_done, mdp.events as mdp_events


@configclass
class EventCfg:
    """Configuration for events."""

    reset_robot_spawn = EventTermCfg(
        func=mdp_events.reset_robot_curriculum_spawn,
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
            usd_path=os.path.join(
                ARCPRO_LAB_DIR, "assets", "robot", "F1Tenth_Metric.usd"
            ),
            scale=(1.0, 1.0, 1.0),  # Revert to 1.0x
        ),
        init_state=ARCPRO_ROBOT_CFG.init_state.replace(
            # Spawn point: right lane heading toward junction_18 intersection.
            # Exactly on the path center (X=-16.197) at safe drop height
            pos=(-16.197, 5.50, 0.5),
            rot=(0.7071, 0.0, 0.0, -0.7071),  # Upright & Face South (WXYZ)
            # No kickstart — proper tire friction + drive torque is sufficient.
            # Kickstart was masking physics issues (bouncing, height terminations).
        ),
    )

    # Camera (Mimic Intel RealSense D455 Ultra-Wide - 115° HFOV, Global Shutter)
    tiled_camera = TiledCameraCfg(
        prim_path="{ENV_REGEX_NS}/Robot/Chassis/CameraSensor",
        update_period=0.0,
        spawn=sim_utils.PinholeCameraCfg(
            horizontal_aperture=6.06,  # 115-degree HFOV (2 * atan(6.06 / (2 * 1.93))) — unlocks inner apex visibility in sharp R=1.16m turns
            focal_length=1.93,
        ),
        # Realistically mounted on F1Tenth front bumper at pos=(0.22, 0.0, 0.18)
        # 15 degrees downward pitch around Y (w=0.9914, y=0.1305) to see road 0.25m to 4.0m ahead
        offset=TiledCameraCfg.OffsetCfg(
            pos=(0.22, 0.0, 0.18), rot=(0.9914, 0.0, 0.1305, 0.0), convention="world"
        ),
        data_types=["rgb"],
        width=224,
        height=224,
    )

    # Contact Sensor: Detect chassis collisions (crashes)
    contact_forces = ContactSensorCfg(
        prim_path="{ENV_REGEX_NS}/Robot/Chassis",
        update_period=0.0,
        history_length=3,
        debug_vis=False,
    )

    # Ground Contact Sensor: Detects physical collision between chassis and ground plane (excluding rolling wheels)
    ground_contact = ContactSensorCfg(
        prim_path="{ENV_REGEX_NS}/Robot/Chassis",
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
        # Note: tiled_camera is accessed directly by SKRLFlattenWrapper via scene.sensors["tiled_camera"].data.output["rgb"]
        # Removing it from ObservationManager eliminates the 3 MB/step clone leak in ObservationManager._obs_buffer.

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
        max_steering_angle=0.68,  # 38.96 deg (enables R_min = 0.408m < Corner 1 R=0.464m)
        offset=0.0,
    )
    b_drive = arcpro_actions.AckermannDifferentialVelocityActionCfg(
        asset_name="robot",
        joint_names=["Joint_Drive_.*"],
        wheelbase=0.33,
        track_width=0.28,
        scale=20.0,
        offset=20.0,
    )


@configclass
class RewardCfg:
    # Survival Bonus: Disabled completely (weight 0.0). We rely entirely on bounded progress reward.
    # A positive survival bonus caused the agent to spin in circles to farm "living time" without making progress.
    survival_bonus = RewTerm(
        func=lambda env: torch.ones(env.num_envs, device=env.device), weight=0.0
    )

    # Anti-Suicide: High penalty (-400) to ensure line crossing completely negates sprint progress
    termination_penalty = RewTerm(func=mdp_rew.termination_penalty, weight=400.0)

    # Bounded Progress Reward (scaled to 50.0 so safety/boundary avoidance dominates)
    progress_reward = RewTerm(
        func=lambda env: torch.tanh(mdp_rew.waypoint_progress_reward(env)), weight=50.0
    )

    # Intersection Teacher Guidance: Rewards matching ideal geometric turn arc & corner speed inside intersection
    intersection_expert_guidance = RewTerm(
        func=mdp_rew.intersection_expert_reward, weight=100.0
    )

    # Pure Vision Lane Centering: Rewards staying equidistant between white and double-yellow lines
    lane_centering = RewTerm(func=mdp_rew.lane_centering_reward, weight=120.0)

    # Precision: Waypoint lateral error reward
    lateral_error = RewTerm(func=mdp_rew.lateral_error_reward, weight=15.0)

    # Force movement: Stagnation penalty
    stationary = RewTerm(func=mdp_rew.stationary_penalty, weight=80.0)

    # Directional Alignment: Forward heading reward
    heading = RewTerm(func=mdp_rew.heading_alignment_reward, weight=80.0)
    smoothness = RewTerm(func=mdp_rew.action_rate_smoothness_reward, weight=0.5)

    # Jitter Suppression (relaxed to allow responsive corner turn-in)
    jerk = RewTerm(func=mdp_rew.jerk_penalty, weight=0.01)

    # Boundary Proximity Penalty: Strong continuous repulsion when tires near yellow/white lines
    boundary = RewTerm(func=mdp_rew.boundary_penalty, weight=60.0)

    # Curvature-Adaptive Target Speed Tracking
    speed_tracking = RewTerm(func=mdp_rew.curvature_speed_reward, weight=180.0)


@configclass
class TerminationCfg:
    roadmark_contact = DoneTerm(func=mdp_done.white_line_contact)
    # Physical USD Contact: Terminate if any part of the robot touches the physical ground plane.
    # NOTE FOR USER: Threshold MUST be > 50.0! The track mesh is thin and rests 5cm above the ground plane.
    ground_contact_term = DoneTerm(
        func=mdp_done.ground_contact_termination,
        params={"sensor_cfg": SceneEntityCfg("ground_contact"), "threshold": 100.0},
    )
    # Hard Height Bounds: Instant reset if car falls through track (Z < -0.05m) or flips/flies (Z > 0.50m)
    height_bounds = DoneTerm(
        func=mdp_done.height_termination,
        params={"threshold": -0.05, "max_height": 0.50},
    )
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
        dt=0.002,  # 500Hz for high-fidelity small scale physics
        render_interval=10,  # Maintain visual sync with decimation (500 / 10 = 50Hz)
        device="cuda:0",
        gravity=(0.0, 0.0, -9.81),  # Explicit Earth Gravity
        physx=sim_utils.PhysxCfg(
            solver_type=1,  # TGS
            max_position_iteration_count=16,
            max_velocity_iteration_count=4,
            bounce_threshold_velocity=0.5,
            enable_ccd=True,
            enable_stabilization=True,
            gpu_max_rigid_contact_count=2**16,  # 64k contacts
            gpu_max_rigid_patch_count=2
            ** 13,  # 8k patches (increased from 2**11 due to patch buffer overflow)
            gpu_heap_capacity=2**26,  # 64MB heap
            gpu_found_lost_pairs_capacity=2**13,
        ),
    )

    def __post_init__(self):
        self.decimation = (
            10  # 500Hz / 10 = 50Hz control (Nyquist stability for 6cm clearance)
        )
        self.episode_length_s = 3000.0
        self.viewer.camera_follow_prim_path = None
        # Enable cameras conditionally
        if not self.enable_cameras:
            self.scene.tiled_camera = None
            if hasattr(self.observations.policy, "tiled_camera"):
                self.observations.policy.tiled_camera = None
