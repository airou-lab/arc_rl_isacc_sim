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

    # Calculate yaw from quaternion
    q = asset.data.root_quat_w
    # yaw = atan2(2(qw*qz + qx*qy), 1 - 2(qy^2 + qz^2))
    yaw = torch.atan2(2.0 * (q[:, 0] * q[:, 3] + q[:, 1] * q[:, 2]), 1.0 - 2.0 * (q[:, 2]**2 + q[:, 3]**2))

    # Get environment origins to convert world pos to local track pos
    env_origins = env.scene.env_origins
    local_pos = asset.data.root_pos_w - env_origins

    # Slot 0: turn_token — still PVP-masked to 0 (sim has no Worker).
    obs[:, 0] = 0.0

    # Slot 1: go_signal — UNMASKED. Driven by GoSignalManager from the camera
    #         feed (T3.2). When go_signal=0 the policy should brake;
    #         T3.3 also enforces this in the action term as a safety gate.
    try:
        from mdp.go_signal_manager import get_go_signal_manager
        mgr = get_go_signal_manager(num_envs=env.num_envs, device=env.device)
        camera = env.scene.get("tiled_camera") if hasattr(env.scene, "get") else None
        if camera is None:
            # Fallback for non-Dict scenes (older IsaacLab).
            try:
                camera = env.scene["tiled_camera"]
            except KeyError:
                camera = None
        images = None
        if camera is not None and hasattr(camera, "data") and "rgb" in camera.data.output:
            images = camera.data.output["rgb"]  # (N, H, W, 3) uint8
        go_signal = mgr.update(images)
        env.extras["go_signal"] = go_signal
        obs[:, 1] = go_signal
    except Exception:
        # Defensive: detector or camera failure must not crash training.
        # Falls back to "always go" (slot 1 stays 0 — telemetry obs init).
        obs[:, 1] = 1.0

    # Slot 2: goal_dist — PVP-masked to 0.
    obs[:, 2] = 0.0

    # Index 3: Forward Speed (m/s) - Absolute Magnitude for Control Flip support
    obs[:, 3] = torch.norm(asset.data.root_lin_vel_b[:, :2], dim=1)

    # Index 4: Yaw Rate (rad/s) - Local Z angular velocity
    obs[:, 4] = asset.data.root_ang_vel_b[:, 2]

    # Indices 5-7: Last Actions (Clipped)
    try:
        if env.action_manager.action is not None and env.action_manager.action.shape[1] >= 3:
            obs[:, 5:8] = env.action_manager.action[:, :3] # Steer, Throttle, Brake
    except:
        pass 

    # Index 8 & 9: Lateral and Heading Error
    from mdp.track_manager import get_track_manager
    tm = get_track_manager(device=env.device)

    # 1. Update persistent track indices and compute errors
    lat_err, head_err, kappa = tm.compute_errors(local_pos, yaw)    
    
    # 2. Get Marker Distances (Yellow, White, Gate)
    dist_y, dist_w, dist_g = tm.compute_marker_distances(local_pos)
    
    # Distance Tracking (Accumulated)
    if "distance" not in env.extras:
        env.extras["distance"] = torch.zeros(env.num_envs, device=env.device)
    
    # Reset distance for environments that just reset
    reset_buf = getattr(env, "reset_buf", None)
    if reset_buf is not None:
        env.extras["distance"] = torch.where(reset_buf, torch.zeros_like(env.extras["distance"]), env.extras["distance"])

    # Calculate distance moved in this step
    env.extras["distance"] += asset.data.root_lin_vel_b[:, 0] * 0.05
    
    # Store raw values in extras for Reward/Termination (Unmasked)
    env.extras["lat_err"] = lat_err
    env.extras["head_err"] = head_err
    env.extras["dist_g"] = dist_g # Masking signal for gate crossing
    env.extras["dist_y"] = dist_y
    env.extras["dist_w"] = dist_w
    
    # Vision-only Mandate: Mask ground-truth errors in the policy observation
    # This forces the ResNet-18 to learn features from the camera
    obs[:, 8] = 0.0 # MASKED lat_err
    obs[:, 9] = 0.0 # MASKED head_err
    
    # Index 10: Path Curvature (Kappa)
    obs[:, 10] = kappa

    # Index 11: Total Distance (Accumulated)
    if "distance" in env.extras:
        obs[:, 11] = env.extras["distance"]
        
    # Final verification for NaNs
    nan_mask = torch.isnan(obs)
    if nan_mask.any():
        obs[nan_mask] = 0.0
        
    return obs
