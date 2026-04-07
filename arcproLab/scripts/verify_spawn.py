# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""
Sanity check script to verify robot spawning and track management.
Usage: python arcproLab/scripts/verify_spawn.py --num_envs 1
"""

import argparse
from isaaclab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(description="Verify robot spawning and track alignment.")
parser.add_argument("--num_envs", type=int, default=1, help="Number of environments to spawn.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest of the script."""

import torch
import numpy as np
import os
import sys
from isaaclab.envs import ManagerBasedRLEnv
# Add both root and arcproLab to sys.path
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, "..", ".."))
ARCPRO_LAB_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, ".."))

if ROOT_DIR not in sys.path:
    sys.path.append(ROOT_DIR)
if ARCPRO_LAB_DIR not in sys.path:
    sys.path.append(ARCPRO_LAB_DIR)

from arcpro_env_cfg import ARCProEnvCfg
from arcproLab.mdp.track_manager import get_track_manager

def main():
    # setup environment configuration
    env_cfg = ARCProEnvCfg()
    env_cfg.scene.num_envs = args_cli.num_envs if args_cli.num_envs else 1
    
    # setup environment
    env = ManagerBasedRLEnv(cfg=env_cfg)
    
    # get track manager
    tm = get_track_manager(device=env.device, num_envs=env.num_envs)
    
    print(f"[Verify] Simulation started with {env.num_envs} environments.")
    print(f"[Verify] Track Manager has {len(tm.waypoints)} waypoints.")
    
    # reset environment
    obs, _ = env.reset()
    
    # check robot positions
    robot_pos = env.scene["robot"].data.root_pos_w
    print(f"[Verify] Initial Robot Position (Env 0): {robot_pos[0].cpu().numpy()}")
    
    # --- DEBUG: Check Track Collisions ---
    import omni.usd
    from pxr import Usd, UsdPhysics, UsdGeom
    stage = omni.usd.get_context().get_stage()
    
    # Check Physics Scene
    physics_scene_path = "/physicsScene"
    phys_prim = stage.GetPrimAtPath(physics_scene_path)
    if phys_prim:
        print(f"[Verify] Found PhysicsScene at {physics_scene_path}")
    else:
        # Check alternative common paths
        for p in stage.Traverse():
            if p.HasAPI(UsdPhysics.SceneAPI):
                print(f"[Verify] Found PhysicsScene (via API) at {p.GetPath()}")
                break
    
    track_prim_path = "/World/envs/env_0/Track"
    track_prim = stage.GetPrimAtPath(track_prim_path)
    if track_prim:
        print(f"[Verify] Found Track Prim at {track_prim_path}")
        # Traverse children to find meshes
        mesh_count = 0
        coll_count = 0
        first_mesh_path = None
        for prim in Usd.PrimRange(track_prim):
            if prim.IsA(UsdGeom.Mesh):
                if first_mesh_path is None: first_mesh_path = prim.GetPath()
                mesh_count += 1
                if prim.HasAPI(UsdPhysics.CollisionAPI):
                    coll_count += 1
        print(f"[Verify] Track has {mesh_count} meshes, {coll_count} with CollisionAPI.")
        
        if first_mesh_path:
            mesh_prim = stage.GetPrimAtPath(first_mesh_path)
            xformable = UsdGeom.Xformable(mesh_prim)
            world_transform = xformable.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
            translation = world_transform.ExtractTranslation()
            
            bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
            bbox = bbox_cache.ComputeWorldBound(mesh_prim)
            range_3d = bbox.ComputeAlignedRange()
            print(f"[Verify] Sample Mesh: {first_mesh_path} | World Translation: {translation}")
            print(f"[Verify] Mesh BBox Min: {range_3d.GetMin()} | Max: {range_3d.GetMax()}")
    else:
        print(f"[Verify] ERR: Track Prim NOT FOUND at {track_prim_path}")
    # --------------------------------------
    
    # check track errors at spawn
    # extract yaw from quaternion
    q = env.scene["robot"].data.root_quat_w
    yaw = torch.atan2(2.0 * (q[:, 0] * q[:, 3] + q[:, 1] * q[:, 2]), 1.0 - 2.0 * (q[:, 2]**2 + q[:, 3]**2))
    
    lat_err, head_err = tm.compute_errors(robot_pos, yaw)
    print(f"[Verify] Lateral Error (Env 0): {lat_err[0].item():.4f}m")
    print(f"[Verify] Heading Error (Env 0): {head_err[0].item():.4f} rad")
    
    # run a few steps of simulation to check for falling/clipping
    print("[Verify] Running 50 steps of simulation...")
    for i in range(50):
        # apply zero actions
        actions = torch.zeros(env.action_manager.action.shape, device=env.device)
        obs, rewards, terminations, truncations, extras = env.step(actions)
        
        if i % 10 == 0:
            curr_pos = env.scene["robot"].data.root_pos_w[0].cpu().numpy()
            print(f" - Step {i:2d}: Position={curr_pos}, Terminated={terminations[0].item()}")
            
    print("[Verify] Sanity check complete.")
    
    # close the simulator
    env.close()

if __name__ == "__main__":
    main()
