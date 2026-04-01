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
    
    # --- Discrete/Signal Data ---
    # Index 0: Turn Token (Discrete turn command {-1, 0, 1})
    obs[:, 0] = env.extras.get("turn_token", torch.zeros(env.num_envs, device=env.device))
    
    # Index 1: Go Signal (Go/wait {0.0, 1.0})
    obs[:, 1] = env.extras.get("go_signal", torch.ones(env.num_envs, device=env.device))
    
    # Index 2: Goal Distance (0.0 for Endless Mode)
    obs[:, 2] = 0.0
    
    # --- Physical Data ---
    # Index 3: Forward Speed (m/s) - Local X velocity
    speed = asset.data.root_lin_vel_b[:, 0]
    obs[:, 3] = speed
    
    # Index 4: Yaw Rate (rad/s) - Local Z angular velocity
    obs[:, 4] = asset.data.root_ang_vel_b[:, 2]
    
    # --- Action History ---
    # Indices 5-7: Last Actions (Clipped)
    # [5]: Steer, [6]: Throttle, [7]: Brake
    if env.action_manager.action is not None:
        # Action shape is (N, 6) -> 2 steer, 4 drive
        obs[:, 5] = env.action_manager.action[:, 0] # Use first steering joint
        obs[:, 6] = env.action_manager.action[:, 2] # Use first drive joint
        # Index 7 (Brake) is usually 0.0 in this simulation setup
        obs[:, 7] = 0.0
    
    # --- Track Error Data ---
    # Use TrackManager for high-fidelity errors (not axis-aligned)
    from mdp.track_manager import get_track_manager
    tm = get_track_manager(device=env.device)
    
    # extract yaw from quaternion
    q = asset.data.root_quat_w
    yaw = torch.atan2(2.0 * (q[:, 0] * q[:, 3] + q[:, 1] * q[:, 2]), 1.0 - 2.0 * (q[:, 2]**2 + q[:, 3]**2))
    
    pos = asset.data.root_pos_w
    lat_err, head_err = tm.compute_errors(pos, yaw)
    obs[:, 8] = lat_err
    obs[:, 9] = head_err
    
    # Index 10: Kappa (Curvature 1/m)
    obs[:, 10] = tm.get_curvature(pos)
    
    # --- Odometry Data ---
    # Index 11: Total Distance (Accumulated)
    if "distance" not in env.extras:
        env.extras["distance"] = torch.zeros(env.num_envs, device=env.device)
    
    # Update distance traveled: speed * dt
    # env.sim_dt is usually the physics step
    env.extras["distance"] += torch.abs(speed) * env.physics_dt
    obs[:, 11] = env.extras["distance"]
        
    # Final verification for NaNs
    nan_mask = torch.isnan(obs)
    if nan_mask.any():
        obs[nan_mask] = 0.0
        
    return obs
