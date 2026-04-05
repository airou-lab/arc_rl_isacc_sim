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
    """Configuration for the F1Tenth robot using the GENERATED primitive-based asset."""
    
    spawn: sim_utils.UsdFileCfg = sim_utils.UsdFileCfg(
        usd_path=os.path.join(os.path.dirname(__file__), "assets", "robot", "F1Tenth_Metric.usd"),
        scale=(1.0, 1.0, 1.0), 
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            retain_accelerations=False,
            linear_damping=0.0,
            angular_damping=0.0,
            max_linear_velocity=1000.0,
            max_angular_velocity=1000.0,
            max_depenetration_velocity=100.0,
        ),
        mass_props=sim_utils.MassPropertiesCfg(mass=20.0), # Realistic 20kg for 8x scale
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False,
            solver_position_iteration_count=32, # Precision solver
            solver_velocity_iteration_count=16, # Precision solver
            fix_root_link=False, # EXPLICITLY UNLOCK ROOT FOR GRAVITY
        ),
    )

    init_state: ArticulationCfg.InitialStateCfg = ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.0), 
        rot=(1.0, 0.0, 0.0, 0.0),
        joint_pos={".*": 0.0},
    )

    actuators: dict = {
        "steering": ImplicitActuatorCfg(
            joint_names_expr=["Joint_Steer_.*"],
            effort_limit_sim=1000.0,
            velocity_limit_sim=10.0,
            stiffness=2000.0, 
            damping=100.0,
        ),
        "throttle": ImplicitActuatorCfg(
            joint_names_expr=["Joint_Drive_.*"],
            effort_limit_sim=10000.0,
            velocity_limit_sim=200.0,
            stiffness=0.0,
            damping=500.0, 
        ),
    }

##
# Configuration
##
ARCPRO_ROBOT_CFG = ArcProRobotCfg()
