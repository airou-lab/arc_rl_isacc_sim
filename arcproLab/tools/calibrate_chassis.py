# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Measure native chassis size.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

from pxr import Usd, UsdGeom, Gf
import os

def main():
    usd_path = "f1tenth_trainer/assets/F1Tenth_Modern.usd"
    stage = Usd.Stage.Open(usd_path)
    
    # Target the main chassis mesh
    # Based on previous inspect, it's likely under Rigid_Bodies/Chassis
    chassis_path = "/Full_Car/Rigid_Bodies/Chassis/Chassis"
    prim = stage.GetPrimAtPath(chassis_path)
    
    if prim:
        bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
        bbox = bbox_cache.ComputeWorldBound(prim)
        size = bbox.GetRange().GetMax() - bbox.GetRange().GetMin()
        print(f"\n[Native] Chassis Mesh Size: {size[0]:.4f} (L) x {size[1]:.4f} (W) units")
        
        # Calculate scale factor for 403mm Length
        target_l = 0.403
        scale_l = target_l / size[0]
        print(f"[Calib] Required Scale for 403mm Length: {scale_l:.6f}")
        
        # Calculate resulting width
        result_w = size[1] * scale_l
        print(f"[Calib] Resulting Width at this scale: {result_w*1000:.2f} mm (Target: 287 mm)")
    else:
        print("[Native] ERR: Could not find chassis mesh.")

    simulation_app.close()

if __name__ == "__main__":
    main()
