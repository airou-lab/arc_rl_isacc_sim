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
    Indices:
    [0]: turn_token (from env.extras or default 0.0)
    [1]: go_signal (from env.extras or default 1.0)
    [2]: goal_dist (0.0 for lane following)
    [3]: forward_speed (m/s)
    [4]: yaw_rate (rad/s)
    [5]: last_steer
    [6]: last_throttle
    [7]: last_brake (0.0)
    [8]: lateral_error (m)
    [9]: heading_error (rad)
    [10]: kappa (curvature from TrackManager)
    [11]: cumulative_distance (m)
    """
    asset = env.scene[asset_cfg.name]
    
    # Initialize observation tensor (batch_size, 12)
    obs = torch.zeros((env.num_envs, 12), device=env.device)
    
    # Index 0 & 1: Mission Logic (Optional)
    obs[:, 0] = env.extras.get("turn_token", torch.zeros(env.num_envs, device=env.device))
    obs[:, 1] = env.extras.get("go_signal", torch.ones(env.num_envs, device=env.device))
    
    # Index 3: Forward Speed (m/s) - Local X velocity
    obs[:, 3] = asset.data.root_lin_vel_b[:, 0]
    
    # Index 4: Yaw Rate (rad/s) - Local Z angular velocity
    obs[:, 4] = asset.data.root_ang_vel_b[:, 2]
    
    # Indices 5-7: Last Actions (Clipped)
    try:
        # Action Manager stores the last applied action
        if hasattr(env.action_manager, "action") and env.action_manager.action is not None:
            # SB3 actions are usually (batch, action_dim)
            # 5: Steer, 6: Throttle
            obs[:, 5:7] = env.action_manager.action[:, :2] 
    except Exception:
        pass # Actions not yet defined/initialized
    
    # Index 8, 9 & 10: Track Related (Errors and Curvature)
    from mdp.track_manager import get_track_manager
    tm = get_track_manager(device=env.device)
    
    # extract yaw from quaternion
    q = asset.data.root_quat_w
    yaw = torch.atan2(2.0 * (q[:, 0] * q[:, 3] + q[:, 1] * q[:, 2]), 1.0 - 2.0 * (q[:, 2]**2 + q[:, 3]**2))
    
    lat_err, head_err = tm.compute_errors(asset.data.root_pos_w, yaw)
    obs[:, 8] = lat_err
    obs[:, 9] = head_err
    
    # Index 10: Kappa (Local Curvature)
    obs[:, 10] = tm.get_curvature(asset.data.root_pos_w)
    
    # Index 11: Total Distance (Accumulated)
    obs[:, 11] = env.extras.get("distance", torch.zeros(env.num_envs, device=env.device))
        
    # Final verification for NaNs
    nan_mask = torch.isnan(obs)
    if nan_mask.any():
        obs[nan_mask] = 0.0
        
    return obs
