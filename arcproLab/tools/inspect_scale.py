# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Inspect spawned prim scale.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import omni.usd
from pxr import Usd, UsdGeom
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
    track_path = "/World/envs/env_0/Track"
    prim = stage.GetPrimAtPath(track_path)
    
    if prim:
        xform = UsdGeom.Xformable(prim)
        ops = xform.GetOrderedXformOps()
        print(f"\n[Inspect] Prim: {track_path}")
        for op in ops:
            print(f" - Op: {op.GetOpName()} | Value: {op.Get()}")
        
        # Check a child mesh
        child_path = "/World/envs/env_0/Track/drivable_surfaces/tiles/pavement_1/ref_pavement_1/road_tile_1/opaque__asphalt__drivable_light_Road/piece_2"
        child_prim = stage.GetPrimAtPath(child_path)
        if child_prim:
            bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
            bbox = bbox_cache.ComputeWorldBound(child_prim)
            size = bbox.GetRange().GetMax() - bbox.GetRange().GetMin()
            print(f"[Inspect] Child Mesh: {child_path}")
            print(f"[Inspect] Child World Size: {size[0]:.2f} x {size[1]:.2f} m")
    else:
        print(f"[Inspect] ERR: Prim {track_path} not found.")

    simulation_app.close()

if __name__ == "__main__":
    main()
