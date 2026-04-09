# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Camera and roadmark diagnostic.")
parser.add_argument("--num_envs", type=int, default=1, help="Number of environments.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../..")))

import torch
import omni.usd
from pxr import UsdGeom, Gf
from isaaclab.envs import ManagerBasedRLEnv
from arcproLab.arcpro_env_cfg import ARCProEnvCfg
import PIL.Image as im

def main():
    env_cfg = ARCProEnvCfg()
    env_cfg.scene.num_envs = 1
    env = ManagerBasedRLEnv(cfg=env_cfg)
    
    stage = omni.usd.get_context().get_stage()
    
    print("\n" + "="*60)
    print("CAMERA & ROADMARK DIAGNOSTIC")
    print("="*60 + "\n")

    env.reset()
    
    # 1. Capture Camera Image
    obs, _, _, _, _ = env.step(torch.zeros(env.action_manager.action.shape, device=env.device))
    
    # Observation shape check:
    # Based on arcpro_env_cfg, 'visual' is (90, 160, 3)
    if "visual" in env.observation_manager.compute():
        img_tensor = env.observation_manager.compute()["visual"][0].cpu()
        # Scale to 0-255 if needed, but Isaac Lab usually returns 0-255 uint8 or 0-1 float
        if img_tensor.dtype == torch.float32:
            img_tensor = (img_tensor * 255).byte()
        
        img_np = img_tensor.numpy()
        img = im.fromarray(img_np)
        img_path = os.path.join(os.path.dirname(__file__), "agent_view.png")
        img.save(img_path)
        print(f"[Info] Saved agent camera view to: {img_path}")
    else:
        print("[Error] 'visual' observation group not found.")

    # 2. Measure Roadmark Mesh Positions
    # We look for roadmark meshes under env_0
    roadmark_widths = []
    for prim in omni.usd.get_context().get_stage().Traverse():
        if prim.IsA(UsdGeom.Mesh) and "roadmarks" in str(prim.GetPath()).lower():
            bbox = UsdGeom.Imageable(prim).ComputeWorldBound(0, "default")
            range = bbox.GetRange()
            min_pt, max_pt = range.GetMin(), range.GetMax()
            # Most roadmarks are narrow strips (the lines themselves)
            dx = max_pt[0] - min_pt[0]
            dy = max_pt[1] - min_pt[1]
            # Print position of a few to see where they are
            center = (min_pt + max_pt) * 0.5
            print(f"Roadmark Found: {prim.GetPath()}")
            print(f"  - Position: X={center[0]:.3f}, Y={center[1]:.3f}")
            print(f"  - Dimensions: DX={dx:.3f}m, DY={dy:.3f}m")
            roadmark_widths.append(dy)
            if len(roadmark_widths) > 5: break

    print("\n" + "="*60)
    env.close()
    simulation_app.close()

if __name__ == "__main__":
    main()
