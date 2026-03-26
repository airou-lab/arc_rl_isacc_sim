# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Deep transform inspection.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import omni.usd
from pxr import Usd, UsdGeom, Gf
import sys
import os
# Add the project root to sys.path
sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))

from isaaclab.scene import InteractiveScene
from isaaclab.sim import SimulationContext, SimulationCfg
from arcproLab.arcpro_env_cfg import ARCProSceneCfg

def main():
    sim_cfg = SimulationCfg(dt=1.0/60.0, device="cuda:0")
    sim = SimulationContext(sim_cfg)
    
    scene_cfg = ARCProSceneCfg(num_envs=1, env_spacing=5.0)
    scene_cfg.tiled_camera = None
    scene = InteractiveScene(scene_cfg)
    
    sim.reset()
    
    stage = omni.usd.get_context().get_stage()
    
    # Target mesh
    mesh_path = "/World/envs/env_0/Track/drivable_surfaces/tiles/pavement_1/ref_pavement_1/road_tile_1/opaque__asphalt__drivable_light_Road/piece_2"
    prim = stage.GetPrimAtPath(mesh_path)
    
    if prim:
        print(f"\n[DeepInspect] Prim: {mesh_path}")
        
        # 1. Local Bounding Box (ignore transforms)
        mesh = UsdGeom.Mesh(prim)
        local_bbox = mesh.ComputeLocalBound(Usd.TimeCode.Default(), UsdGeom.Tokens.default_)
        local_size = local_bbox.GetRange().GetMax() - local_bbox.GetRange().GetMin()
        print(f"[DeepInspect] Native Local Size: {local_size[0]:.2f} x {local_size[1]:.2f} m")
        
        # 2. World Bounding Box
        bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
        world_bbox = bbox_cache.ComputeWorldBound(prim)
        world_size = world_bbox.GetRange().GetMax() - world_bbox.GetRange().GetMin()
        print(f"[DeepInspect] Computed World Size: {world_size[0]:.2f} x {world_size[1]:.2f} m")
        
        # 3. Hierarchy Scale Check
        curr = prim
        print("\n[DeepInspect] Hierarchy Scale Trace:")
        while curr and curr.GetPath() != "/World":
            xform = UsdGeom.Xformable(curr)
            # Find scale op
            scale = (1,1,1)
            for op in xform.GetOrderedXformOps():
                if op.GetOpType() == UsdGeom.XformOp.TypeScale:
                    scale = op.Get()
            
            reset = xform.GetResetXformStack()
            print(f" - {curr.GetPath()} | Scale: {scale} | ResetStack: {reset}")
            curr = curr.GetParent()
            
    else:
        print(f"[DeepInspect] ERR: Prim {mesh_path} not found.")

    simulation_app.close()

if __name__ == "__main__":
    main()
