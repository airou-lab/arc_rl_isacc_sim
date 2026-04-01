# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import torch
from isaaclab.managers import SceneEntityCfg
from isaaclab.envs import ManagerBasedRLEnv

def get_telemetry_vector(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    """
    Constructs the 12-element telemetry vector for the ARCPro policy.
    Protocol:
    - [0]: turn_token (0.0)
    - [1]: go_signal (1.0)
    - [2]: goal_dist (0.0 - Endless Mode)
    - [3]: forward_speed (m/s)
    - [4]: yaw_rate (rad/s)
    - [5]: last_steer
    - [6]: last_throttle
    - [7]: last_brake (0.0)
    - [8]: lateral_error (m)
    - [9]: heading_error (rad)
    - [10]: kappa (Curvature)
    - [11]: cumulative_distance (m)
    """
    asset = env.scene[asset_cfg.name]
    
    # Initialize observation tensor (batch_size, 12)
    obs = torch.zeros((env.num_envs, 12), device=env.device)
    
    # Fixed signals
    obs[:, 1] = 1.0 # go_signal
    
    # Index 3: Forward Speed (m/s) - Local X velocity
    obs[:, 3] = asset.data.root_lin_vel_b[:, 0]
    
    # Index 4: Yaw Rate (rad/s) - Local Z angular velocity
    obs[:, 4] = asset.data.root_ang_vel_b[:, 2]
    
    # Indices 5-6: Last Actions (Clipped)
    if env.action_manager.action is not None:
        # Assuming action space is [steer, throttle]
        obs[:, 5] = env.action_manager.action[:, 0]
        obs[:, 6] = env.action_manager.action[:, 1]
    
    # Track Errors and Curvature
    from mdp.track_manager import get_track_manager
    tm = get_track_manager(device=env.device)
    
    # extract yaw from quaternion
    q = asset.data.root_quat_w
    yaw = torch.atan2(2.0 * (q[:, 0] * q[:, 3] + q[:, 1] * q[:, 2]), 1.0 - 2.0 * (q[:, 2]**2 + q[:, 3]**2))
    
    lat_err, head_err = tm.compute_errors(asset.data.root_pos_w, yaw)
    obs[:, 8] = lat_err
    obs[:, 9] = head_err
    
    # Index 10: Kappa
    obs[:, 10] = tm.get_kappa(asset.data.root_pos_w)
    
    # Index 11: Total Distance (Accumulated)
    if "distance" in env.extras:
        obs[:, 11] = env.extras["distance"]
        
    # Final verification for NaNs
    nan_mask = torch.isnan(obs)
    if nan_mask.any():
        nan_indices = torch.where(nan_mask.any(dim=0))[0]
        # Just zero out NaNs to keep training alive
        obs[nan_mask] = 0.0
        
    return obs
