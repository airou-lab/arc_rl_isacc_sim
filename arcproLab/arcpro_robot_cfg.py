# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import os
import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.utils import configclass

import mdp.spawner as arcpro_spawner

def spawn_f1tenth_preset(prim_path, cfg, translation=None, orientation=None):
    """Preset spawner that includes mass overrides for F1Tenth."""
    mass_overrides = {
        "Chassis": 3.342,
        "Wheel_.*": 0.15,
        "Knuckle_.*": 0.075,
    }
    return arcpro_spawner.spawn_f1tenth(prim_path, cfg, translation, orientation, mass_overrides=mass_overrides)

@configclass
class ArcProRobotCfg(ArticulationCfg):
    """Configuration for the F1Tenth robot using the GENERATED primitive-based asset."""
    
    spawn: sim_utils.UsdFileCfg = sim_utils.UsdFileCfg(
        func=spawn_f1tenth_preset,
        usd_path=os.path.join(os.path.dirname(__file__), "assets", "robot", "F1Tenth_Metric.usd"),
        scale=(1.0, 1.0, 1.0), 
        activate_contact_sensors=True,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            retain_accelerations=False,
            linear_damping=0.1,    # RC car rolling resistance + light aero drag
            angular_damping=0.05,  # Gentle yaw/pitch/roll damping — prevents spin-out
            max_linear_velocity=1000.0,
            max_angular_velocity=1000.0,
            max_depenetration_velocity=10.0, # Reduced from 100.0 to prevent joints from snapping/dislocating during clipping
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False,
            solver_position_iteration_count=16, 
            solver_velocity_iteration_count=8, 
            fix_root_link=False, 
        ),
        collision_props=sim_utils.CollisionPropertiesCfg(
            contact_offset=0.005, # 5mm offset for high precision
            rest_offset=0.0, 
        ),
    )

    init_state: ArticulationCfg.InitialStateCfg = ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.1), # Lowered for 1x scale
        rot=(1.0, 0.0, 0.0, 0.0),
        joint_pos={".*": 0.0},
    )

    actuators: dict = {
        "steering": ImplicitActuatorCfg(
            joint_names_expr=["Joint_Steer_.*"],
            effort_limit_sim=10.0, 
            velocity_limit_sim=10.0,
            stiffness=20.0, 
            damping=2.0, 
        ),
        "throttle": ImplicitActuatorCfg(
            joint_names_expr=["Joint_Drive_.*"], 
            effort_limit_sim=1.0, # Middle ground: 1.0 (4 Nm total) to prevent backflips but maintain drive.
            velocity_limit_sim=100.0,
            stiffness=0.0,
            damping=1.0, 
            armature=0.01,
        ),
    }


##
# Configuration
##
ARCPRO_ROBOT_CFG = ArcProRobotCfg()
