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
    Indices follow the legacy protocol established in Phase 1.
    """
    asset = env.scene[asset_cfg.name]
    
    # Initialize observation tensor (batch_size, 12)
    obs = torch.zeros((env.num_envs, 12), device=env.device)
    
    # Index 3: Forward Speed (m/s) - Local X velocity
    # Metric scale: 1.0 = 1m/s
    obs[:, 3] = asset.data.root_lin_vel_b[:, 0]
    
    # Index 4: Yaw Rate (rad/s) - Local Z angular velocity
    obs[:, 4] = asset.data.root_ang_vel_b[:, 2]
    
    # Indices 5-7: Last Actions (Clipped)
    try:
        if env.action_manager.action is not None and env.action_manager.action.shape[1] >= 2:
            obs[:, 5:7] = env.action_manager.action[:, :2] # Steer, Throttle
    except:
        pass # Actions not yet defined in Wave 1
    
    # Index 8 & 9: Lateral and Heading Error
    # Use TrackManager for high-fidelity errors (not axis-aligned)
    from mdp.track_manager import get_track_manager
    tm = get_track_manager(device=env.device)
    
    # extract yaw from quaternion
    q = asset.data.root_quat_w
    # yaw = atan2(2(qw*qz + qx*qy), 1 - 2(qy^2 + qz^2))
    yaw = torch.atan2(2.0 * (q[:, 0] * q[:, 3] + q[:, 1] * q[:, 2]), 1.0 - 2.0 * (q[:, 2]**2 + q[:, 3]**2))
    
    # Get environment origins to convert world pos to local track pos
    env_origins = env.scene.env_origins
    local_pos = asset.data.root_pos_w - env_origins
    
    lat_err, head_err = tm.compute_errors(local_pos, yaw)
    
    # Distance Tracking
    if "distance" not in env.extras:
        env.extras["distance"] = torch.zeros(env.num_envs, device=env.device)
    
    # Calculate distance moved in this step (Forward velocity * dt)
    # We use local X velocity * environment dt (0.05s)
    env.extras["distance"] += asset.data.root_lin_vel_b[:, 0] * 0.05
    
    # Lateral Error (meters relative to Yellow Centerline)
    obs[:, 8] = lat_err
    obs[:, 9] = head_err
    
    # Index 11: Total Distance (Accumulated)
    if "distance" in env.extras:
        obs[:, 11] = env.extras["distance"]
        
    # Store raw values in extras for easy access by scripts (not used by policy)
    env.extras["raw_lat_err"] = lat_err
    env.extras["raw_speed"] = asset.data.root_lin_vel_b[:, 0]
    
    # Final verification for NaNs
    nan_mask = torch.isnan(obs)
    if nan_mask.any():
        nan_indices = torch.where(nan_mask.any(dim=0))[0]
        # Just zero out NaNs to keep training alive
        obs[nan_mask] = 0.0
        
    return obs
