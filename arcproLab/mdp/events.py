# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import torch
from isaaclab.managers import SceneEntityCfg
from isaaclab.envs import ManagerBasedRLEnv

from mdp.track_manager import get_track_manager
import omni.physx
import omni.physx.scripts.utils as physx_utils

def reset_robot_to_fixed_spawn(env: ManagerBasedRLEnv, env_ids: torch.Tensor, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")):
    """
    Event to reset the robot to a FIXED starting waypoint, snapped to the road surface.
    """
    asset = env.scene[asset_cfg.name]
    
    # Spawn point: right lane heading toward junction_18 intersection.
    # X shifted from prior centerline value (-16.25375) -> -15.73 (a ~+0.52 m
    # offset) to center the body in the right lane. Verified visually via
    # inspect_world.py. Keep ARCPRO_ROBOT_CFG.init_state.pos in env_cfg in sync.
    local_spawn_x, local_spawn_y = -15.73, 5.56
    spawn_yaw = 1.5708 # +90 degrees (heading +Y); conforms to policy IsaacDirectConfig.spawn_yaw (OM 3.3-A)
    
    # Get environment origins
    env_origins = env.scene.env_origins[env_ids]
    
    # Initialize tensors
    final_pos = torch.zeros((len(env_ids), 3), device=env.device)
    final_pos[:, 0] = env_origins[:, 0] + local_spawn_x
    final_pos[:, 1] = env_origins[:, 1] + local_spawn_y
    final_pos[:, 2] = env_origins[:, 2] + 0.1 # Lower safe initial height
    
    quats = torch.zeros((len(env_ids), 4), device=env.device)
    import math
    half_yaw = spawn_yaw / 2.0
    quats[:, 0] = math.cos(half_yaw)
    quats[:, 3] = math.sin(half_yaw)
    
    # Raycast interface to find the ground
    query = omni.physx.get_physx_scene_query_interface()
    
    for i, env_id in enumerate(env_ids):
        # Raycast from far above down to the road using the calculated world-space position
        world_x, world_y = final_pos[i, 0].item(), final_pos[i, 1].item()
        hit = query.raycast_closest((world_x, world_y, 100.0), (0.0, 0.0, -1.0), 200.0)
        
        if hit["hit"]:
            hit_path = str(hit.get("rigidBody") or hit.get("collisionPath") or "")
            if "robot" not in hit_path.lower():
                final_pos[i, 2] = hit["position"][2] + 0.1 # Elevated height (Lowered)
                # if i == 0: print(f"[Event] Reset Snapped Env 0 to Z={final_pos[i, 2]:.2f}")
    
    # Teleport
    asset.write_root_pose_to_sim(torch.cat([final_pos, quats], dim=-1), env_ids=env_ids)
    asset.write_root_velocity_to_sim(torch.zeros((len(env_ids), 6), device=env.device), env_ids=env_ids)
    
    # Zero joint velocities to prevent wheels from spinning on spawn
    joint_pos = asset.data.default_joint_pos[env_ids]
    joint_vel = torch.zeros_like(asset.data.joint_vel[env_ids])
    asset.write_joint_state_to_sim(joint_pos, joint_vel, env_ids=env_ids)

    # Reset the cumulative distance obs accumulator for these envs (F2).
    # observations.py slot 11 (`distance`) integrates root_lin_vel_b[:,0]*0.05
    # every step and was previously never zeroed across resets — it drifted
    # unbounded and VecNormalize's running stats never stabilized. Zero it
    # only for the env_ids being reset (preserves accumulators for others
    # in the same vec batch).
    if "distance" in env.extras:
        env.extras["distance"][env_ids] = 0.0

    # Reset the go_signal FSM for these envs (T3.2). On reset the robot is
    # placed behind the same stop bar it had been past; without a reset,
    # the FSM would still be in DEPART or STOP state from the prior episode
    # and the policy would inherit a stale go_signal.
    try:
        from mdp.go_signal_manager import get_go_signal_manager
        mgr = get_go_signal_manager(num_envs=env.num_envs, device=env.device)
        mgr.reset(env_ids)
    except Exception:
        pass


