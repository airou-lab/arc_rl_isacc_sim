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
    
    def __post_init__(self):
        self.spawn = sim_utils.UsdFileCfg(
            usd_path=os.path.join(os.path.dirname(__file__), "..", "f1tenth_trainer", "assets", "F1Tenth_Generated.usd"),
            scale=(1.0, 1.0, 1.0), 
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                disable_gravity=False,
                retain_accelerations=False,
                linear_damping=0.5,
                angular_damping=0.5,
                max_linear_velocity=1000.0,
                max_angular_velocity=1000.0,
                max_depenetration_velocity=1.0,
            ),
            articulation_props=sim_utils.ArticulationRootPropertiesCfg(
                enabled_self_collisions=False,
                solver_position_iteration_count=8,
                solver_velocity_iteration_count=8,
            ),
        )

        self.init_state = ArticulationCfg.InitialStateCfg(
            pos=(0.0, 0.0, 0.0), # Spawn at origin, will be moved by ARCProEnvCfg
            rot=(1.0, 0.0, 0.0, 0.0),
            joint_pos={".*": 0.0},
        )

        # Actuator settings - Matching Generated asset joint names
        self.actuators = {
            "steering": ImplicitActuatorCfg(
                joint_names_expr=["Joint_Steer_.*"],
                effort_limit_sim=100.0,
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

##
# Configuration
##
ARCPRO_ROBOT_CFG = ArcProRobotCfg()
