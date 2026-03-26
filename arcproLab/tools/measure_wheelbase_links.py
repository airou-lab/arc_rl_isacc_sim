# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Measure native wheelbase from links.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

from pxr import Usd, UsdGeom, Gf
import numpy as np

def main():
    usd_path = "f1tenth_trainer/assets/F1Tenth_Modern.usd"
    stage = Usd.Stage.Open(usd_path)
    
    # Links representing the axles
    link_keywords = ["Knuckle_Front", "Upright_Rear"]
    
    positions = {}
    for prim in stage.Traverse():
        if any(key in prim.GetName() for key in link_keywords):
            xform = UsdGeom.Xformable(prim)
            local_to_world = xform.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
            translation = local_to_world.ExtractTranslation()
            positions[prim.GetPath().pathString] = np.array(translation)
            print(f"[Native] Link {prim.GetPath().pathString}: {translation}")
            
    # Calculate averages for front and rear
    front_pos = [v for k, v in positions.items() if "Knuckle_Front" in k]
    rear_pos = [v for k, v in positions.items() if "Upright_Rear" in k]
    
    if front_pos and rear_pos:
        front_axle = np.mean(front_pos, axis=0)
        rear_axle = np.mean(rear_pos, axis=0)
        
        wheelbase = np.linalg.norm(front_axle - rear_axle)
        print(f"\n[Native] Native Wheelbase: {wheelbase:.4f} units")
        
        # Track width (approx from front)
        if len(front_pos) >= 2:
            track_width = np.linalg.norm(front_pos[0] - front_pos[1])
            print(f"[Native] Native Track Width: {track_width:.4f} units")
    else:
        print("[Native] ERR: Could not identify axle links.")

    simulation_app.close()

if __name__ == "__main__":
    main()
