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
        activate_contact_sensors=True,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            retain_accelerations=False,
            linear_damping=0.0,
            angular_damping=0.0,
            max_linear_velocity=1000.0,
            max_angular_velocity=1000.0,
            max_depenetration_velocity=100.0,
        ),
        mass_props=sim_utils.MassPropertiesCfg(mass=20.0), # Realistic 20kg for this scale
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False,
            solver_position_iteration_count=8, 
            solver_velocity_iteration_count=4, 
            fix_root_link=False, 
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
            stiffness=400.0, 
            damping=10.0,
        ),
        "throttle": ImplicitActuatorCfg(
            joint_names_expr=["Joint_Drive_F.*"], # FWD Transition
            effort_limit_sim=4000.0, # Increased from 2000 to handle 20kg mass with only 2 drive wheels
            velocity_limit_sim=100.0,
            stiffness=0.0,
            damping=5.0, 
        ),
        "passive_rear_wheels": ImplicitActuatorCfg(
            joint_names_expr=["Joint_Drive_R.*"], # Keep rear wheels passive
            effort_limit_sim=0.0,
            velocity_limit_sim=0.0,
            stiffness=0.0,
            damping=1.0, # Low damping to allow rolling
        ),
    }

##
# Configuration
##
ARCPRO_ROBOT_CFG = ArcProRobotCfg()
