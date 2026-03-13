# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import torch
from isaaclab.managers import SceneEntityCfg
from isaaclab.envs import ManagerBasedRLEnv

def setup_robot_stability(env: ManagerBasedRLEnv, env_ids: torch.Tensor, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")):
    """
    Event to stabilize the robot by forcing mass and joint properties.
    Ported from Stability Pass (Isaac Sim 2025).
    """
    asset = env.scene[asset_cfg.name]
    
    # Resolve env_ids if None (startup mode)
    if env_ids is None:
        env_ids = torch.arange(env.num_envs, device=env.device)
    
    # Apply to all joints initially
    stiffness = torch.full((len(env_ids), asset.num_joints), 1000.0, device=env.device)
    damping = torch.full((len(env_ids), asset.num_joints), 50.0, device=env.device)
    
    # 3. Skip Drive Wheels (Velocity Joints)
    # Drive wheels must have 0 stiffness to allow velocity control
    drive_joint_names = [
        "Wheel__Knuckle__Front_Left", 
        "Wheel__Knuckle__Front_Right", 
        "Wheel__Upright__Rear_Left", 
        "Wheel__Upright__Rear_Right"
    ]
    for name in drive_joint_names:
        idx, _ = asset.find_joints(name)
        stiffness[:, idx] = 0.0
        damping[:, idx] = 100.0 # High damping for speed stability
    
    # Apply to simulation
    asset.write_joint_stiffness_to_sim(stiffness, env_ids=env_ids)
    asset.write_joint_damping_to_sim(damping, env_ids=env_ids)
    
    # Note: Throttle joints (drive wheels) should have 0 stiffness for velocity control,
    # but for stability we might want some damping. 
    # However, Isaac Lab's ImplicitActuator handles this if configured.
    # This event is a "Hard Override" to ensure USD defaults are ignored.
