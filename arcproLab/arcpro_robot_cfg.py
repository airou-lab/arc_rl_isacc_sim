# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import os
import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.utils import configclass

@configclass
class ArcProRobotCfg(ArticulationCfg):
    """Configuration for the F1Tenth robot (Base Config)."""
    
    prim_path: str = "{ENV_REGEX_NS}/Robot"
    
    spawn = sim_utils.UsdFileCfg(
        usd_path=os.path.join(os.path.dirname(__file__), "..", "f1tenth_trainer", "assets", "F1Tenth_Generated.usd"),
        scale=(1.0, 1.0, 1.0), 
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            linear_damping=0.5,
            angular_damping=0.5,
            max_depenetration_velocity=1.0,
        ),
        collision_props=sim_utils.CollisionPropertiesCfg(
            collision_enabled=True,
            contact_offset=0.005,
            rest_offset=0.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False,
            solver_position_iteration_count=8,
            solver_velocity_iteration_count=8,
        ),
    )

    init_state = ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.0),
        rot=(1.0, 0.0, 0.0, 0.0),
        joint_pos={".*": 0.0},
    )

    actuators = {
        "steering": ImplicitActuatorCfg(
            joint_names_expr=["Joint_Steer_.*"],
            effort_limit_sim=1000.0,
            velocity_limit_sim=10.0,
            stiffness=1000.0,
            damping=10.0,
        ),
        "throttle": ImplicitActuatorCfg(
            joint_names_expr=["Joint_Drive_.*"],
            effort_limit_sim=1000.0,
            velocity_limit_sim=100.0,
            stiffness=0.0,
            damping=10.0, 
        ),
    }

ARCPRO_ROBOT_CFG = ArcProRobotCfg()
